/*
 * A simple DMA Engine driver for Allwinner F series SoC
 *
 * Author: IotaHydrae <writeforever@foxmail.com>
 *
 * 2022 (c) IotaHydrae.  This file is licensed under the terms of the GNU
 * General Public License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/reset.h>
#include <linux/list.h>

#include "virt-dma.h"

/* So, Let's take a look at the DMA system of Suniv SoC,
 *
 * First, There are two kinds of system DMA in the Soc.
 * One is Normal DMA (NDMA) with 4 channels;
 * the other is Dedicated DMA (DDMA) with 4 channels;
 *
 * Normal DMA master interface support single and INCR4
 * operation (may be early terminated), and will treat any
 * response from AHB bus as OK response.
 * Dedicated DMA master interface support single and INCR4
 * operation (may be early terminated), and will treat any
 * response from DMA bus as OK response.
 */

#define DRV_NAME "suniv-dma"

/* Suniv DMA Register offsets */
#define SUNIV_DMA_NDMA_BASE         0x100
#define SUNIV_DMA_DDMA_BASE         0x300
#define SUNIV_DMA_CHN_OFFSET(n)     (n*0x20)
#define SUNIV_NDMA_CHN_REG_OFFSET(v,n)   (v + SUNIV_DMA_NDMA_BASE + \
                                          SUNIV_DMA_CHN_OFFSET(n))
#define SUNIV_DDMA_CHN_REG_OFFSET(v,n)   (v + SUNIV_DMA_DDMA_BASE + \
                                          SUNIV_DMA_CHN_OFFSET(n))
/* Global Registers Map of Suniv DMA */
#define SUNIV_DMA_INT_CTRL_REG      0x00
/* Bits of Interrupt control register */
#define SUNIV_DDMA_FULL_TRANS_INT_EN(n)    BIT(16+2*n+1)
#define SUNIV_DDMA_HALF_TRANS_INT_EN(n)    BIT(16+2*n)
#define SUNIV_NDMA_FULL_TRANS_INT_EN(n)    BIT(2*n+1)
#define SUNIV_NDMA_HALF_TRANS_INT_EN(n)    BIT(2*n)

#define SUNIV_DMA_INT_STA_REG       0x04
/* Bits of Interrupt status register */
#define SUNIV_DDMA_FULL_TRANS_INT_PENDING(n)   BIT(16+2*n+1)
#define SUNIV_DDMA_HALF_TRANS_INT_PENDING(n)   BIT(16+2*n)
#define SUNIV_NDMA_FULL_TRANS_INT_PENDING(n)   BIT(2*n+1)
#define SUNIV_NDMA_HALF_TRANS_INT_PENDING(n)   BIT(2*n)

#define SUNIV_DMA_PTY_CFG_REG       0x08
/* Bits of Priority configure register */
#define SUNIV_DMA_AUTO_LOCK_GATING  BIT(16)
#define SUNIV_DMA_NDMA_PRIORITY_COUNTER_OFFSET   7
#define SUNIV_DMA_AC320_PRIORITY_COUNTER_OFFSET  4


/* Channel Registers Map of Suniv DMA */
#define SUNIV_NDMA_CFG_REG(n)       SUNIV_NDMA_CHN_REG_OFFSET(0x00, n)
#define SUNIV_NDMA_SRC_ADR_REG(n)   SUNIV_NDMA_CHN_REG_OFFSET(0x04, n)
#define SUNIV_NDMA_DES_ADR_REG(n)   SUNIV_NDMA_CHN_REG_OFFSET(0x08, n)
#define SUNIV_NDMA_BYTE_CNT_REG(n)  SUNIV_NDMA_CHN_REG_OFFSET(0x0c, n)

#define SUNIV_DDMA_CFG_REG(n)       SUNIV_DDMA_CHN_REG_OFFSET(0x00, n)
#define SUNIV_DDMA_SRC_ADR_REG(n)   SUNIV_DDMA_CHN_REG_OFFSET(0x04, n)
#define SUNIV_DDMA_DES_ADR_REG(n)   SUNIV_DDMA_CHN_REG_OFFSET(0x08, n)
#define SUNIV_DDMA_BYTE_CNT_REG(n)  SUNIV_DDMA_CHN_REG_OFFSET(0x0c, n)
#define SUNIV_DDMA_PAR_REG(n)       SUNIV_DDMA_CHN_REG_OFFSET(0x18, n)
#define SUNIV_DDMA_GEN_DATA(n)      SUNIV_DDMA_CHN_REG_OFFSET(0x1c, n)

/* Bits of DMA Configure Register */
#define SUNIV_DMA_LOADING               BIT(31)
#define SUNIV_DMA_BUSY                  BIT(30)
#define SUNIV_DMA_CONTINUOUS_MODE_EN    BIT(29)
#define SUNIV_DMA_WAIT_STATE_OFFSET     26
#define SUNIV_DMA_DES_DATAWIDTH_OFFSET  24
#define SUNIV_DMA_DES_BRUST_LEN_OFFSET  23
#define SUNIV_DMA_DES_ADDR_TYPE_OFFSET  21
#define SUNIV_DMA_DES_DRQ_TYPE_OFFSET   16
#define SUNIV_DMA_REMAIN_BYTE_CNT_READ_EN  BIT(15)
#define SUNIV_DMA_SRC_DATAWIDTH_OFFSET  8
#define SUNIV_DMA_SRC_BRUST_LEN_OFFSET  7
#define SUNIV_DMA_SRC_ADDR_TYPE_OFFSET  5
#define SUNIV_DMA_SRC_DRQ_TYPE_OFFSET   0

/* bit field of DMA Configure Register */
#define SUNIV_DMA_DES_DATAWIDTH(width)  ((width) << SUNIV_DMA_DES_DATAWIDTH_OFFSET)
#define SUNIV_DMA_DES_BRUST_LEN(len)    ((len)  << SUNIV_DMA_DES_BRUST_LEN_OFFSET)
#define SUNIV_DMA_DES_ADDR_TYPE(type)   ((type) << SUNIV_DMA_DES_ADDR_TYPE_OFFSET)
#define SUNIV_DMA_DES_DRQ_TYPE(type)    ((type) << SUNIV_DMA_DES_DRQ_TYPE_OFFSET)

#define SUNIV_DMA_SRC_DATAWIDTH(width)  ((width) << SUNIV_DMA_SRC_DATAWIDTH_OFFSET)
#define SUNIV_DMA_SRC_BRUST_LEN(len)    ((len)  << SUNIV_DMA_SRC_BRUST_LEN_OFFSET)
#define SUNIV_DMA_SRC_ADDR_TYPE(type)   ((type) << SUNIV_DMA_SRC_ADDR_TYPE_OFFSET)
#define SUNIV_DMA_SRC_DRQ_TYPE(type)    ((type) << SUNIV_DMA_SRC_DRQ_TYPE_OFFSET)

/* Suniv DMA direction */
#define SUNIV_DMA_MEM_TO_MEM        0x00
#define SUNIV_DMA_MEM_TO_DEV        0x01
#define SUNIV_DMA_DEV_TO_MEM        0x02
#define SUNIV_DMA_DEV_TO_DEV        0x03

#define SUNIV_DMA_NDMA_NR_CHANNELS  4
#define SUNIV_DMA_DDMA_NR_CHANNELS  4
#define SUNIV_DMA_MAX_CHANNELS      (SUNIV_DMA_NDMA_NR_CHANNELS + \
                                     SUNIV_DMA_DDMA_NR_CHANNELS)

#define SUNIV_DMA_MAX_BURST         4

#define SUNIV_DMA_DRQ_TYPE_MAX       GENMASK(4, 0)
#define SUNIV_NDMA_BYET_CNT_MASK     GENMASK(17, 0)
#define SUNIV_DDMA_BYET_CNT_MASK     GENMASK(24, 0)
#define SUNIV_DDMA_DRQ_TYPE_SDRAM   0x1
#define SUNIV_NDMA_DRQ_TYPE_SDRAM   0x11

enum suniv_dma_width {
    SUNIV_DMA_1_BYTE = 0x00,
    SUNIV_DMA_2_BYTE = BIT(1),
    SUNIV_DMA_4_BYTE = BIT(2),
};

enum suniv_dma_busrt_len {
    SUNIV_DMA_BURST_1 = 0x00,
    SUNIV_DMA_BURST_4 = BIT(1),
};

enum suniv_dma_addr_type {
    SUNIV_DMA_ADDR_TYPE_LINER = 0x00,
    SUNIV_DMA_ADDR_TYPE_IO    = BIT(1),
};

/*
 * Hardware configuration of dma channel
 */
struct suniv_dma_chn_hw {
    u32 busy;
    dma_addr_t src_addr;
    dma_addr_t des_addr;
    u32 datawidth;
    u32 brust_len;
    u32 byte_cnt;

    u32 cfg;
};

/*
 * TODO: This scatter-gather need a link work list
 * to handle dma prompts, we still need this.
 */
struct suniv_dma_prompt {
    struct suniv_dma_chn_hw hw;
    struct list_head list;
};

struct suniv_dma_diagnosis {
    u32 cid;
    struct suniv_dma_chn_hw reg;
};

/*
 * description of current dma channel
 */
struct suniv_dma_desc {
    u32 remain_bytes;

    struct suniv_dma_chn_hw      chan_hw;
    struct virt_dma_desc         vdesc;

    enum dma_transfer_direction  dir;
};

/* this struct stores everything we need for dma work */
struct suniv_dma_chan {
    u32 id;         /* channel id */
    u8 endpoint;    /* actually means DRQ type */
    bool is_dedicated;
    bool trans_done;

    struct virt_dma_chan    vchan; /* current virtual channel servicing */
    struct suniv_dma_desc   *desc;
    struct dma_slave_config slave_cfg;  /* slave config we received */
};

struct suniv_dma {
    void __iomem            *base;      /* Memory address base of DMA controller */
    int                     irq;

    spinlock_t              lock;
    struct mutex            mutex;

    struct dma_device       ddev;

    struct clk              *hclk;
    struct clk              *mclk;
    struct reset_control    *rstc;

    struct suniv_dma_chan   chan[SUNIV_DMA_MAX_CHANNELS];
    u32                     nr_channels;

    struct suniv_dma_prompt     prompts;

    struct suniv_dma_diagnosis  diag;    /* for dev attr */
};

static inline u32 suniv_dma_read(struct suniv_dma *dma_dev, u32 reg)
{
    return readl(dma_dev->base + reg);
}

static inline void suniv_dma_write(struct suniv_dma *dma_dev, u32 reg, u32 val)
{
    writel(val, dma_dev->base + reg);
}

static inline void suniv_dma_update(struct suniv_dma *dma_dev, u32 reg,
                                    u32 mask, u32 val)
{
    u32 dump_val = readl(dma_dev->base + reg);
    u32 tmp;

    tmp = (dump_val & ~mask) | val;
    writel(tmp, dma_dev->base + reg);
}

static struct device *chan2dev(struct dma_chan *chan)
{
    return &chan->dev->device;
}

static inline struct suniv_dma_chan *to_suniv_dma_chan(struct dma_chan *c)
{
    return container_of(c, struct suniv_dma_chan, vchan.chan);
}

static inline struct suniv_dma *to_suniv_dma_by_dc(struct dma_chan *c)
{
    return container_of(c->device, struct suniv_dma, ddev);
}

static inline struct suniv_dma *to_suniv_dma_by_sdc(struct suniv_dma_chan *c)
{
    return container_of(c->vchan.chan.device, struct suniv_dma, ddev);
}

static inline struct suniv_dma_desc *to_suniv_dma_desc(struct virt_dma_desc *vd)
{
    return container_of(vd, struct suniv_dma_desc, vdesc);
}

static int suniv_dma_enable(struct suniv_dma *dma_dev)
{
    int rc;

    rc = clk_prepare_enable(dma_dev->hclk);
    if (rc) {
        dev_err(dma_dev->ddev.dev, "can't enable AHB clock!\n");
        return rc;
    }

    rc = clk_prepare_enable(dma_dev->mclk);
    if (rc) {
        dev_err(dma_dev->ddev.dev, "can't enable module clock!\n");
        return rc;
    }

    rc = reset_control_assert(dma_dev->rstc);
    if (rc) {
        dev_err(dma_dev->ddev.dev, "can't assert the device from device!\n");
        return rc;
    };

    /* A reset is in need */
    pr_debug("%s: reset the dma controller\n", __func__);
    reset_control_reset(dma_dev->rstc);

    /* initialize interrupt config */
    suniv_dma_write(dma_dev, SUNIV_DMA_INT_CTRL_REG, 0);
    suniv_dma_write(dma_dev, SUNIV_DMA_INT_STA_REG, 0xFFFFFFFF);

    return rc;
}

static void suniv_dma_disable(struct suniv_dma *dma_dev)
{
    clk_disable_unprepare(dma_dev->hclk);
    clk_disable_unprepare(dma_dev->mclk);

    reset_control_assert(dma_dev->rstc);
}

static void __maybe_unused suniv_dma_reg_dump(struct suniv_dma_chan *sdc)
{
    u32 reg_cfg, reg_intc, reg_intp;
    u32 reg_src, reg_dst, reg_byte;
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(sdc);

    reg_intc = suniv_dma_read(dma_dev, SUNIV_DMA_INT_CTRL_REG);
    reg_intp = suniv_dma_read(dma_dev, SUNIV_DMA_INT_STA_REG);

    if (sdc->is_dedicated) {
        reg_cfg = suniv_dma_read(dma_dev, SUNIV_DDMA_CFG_REG(sdc->id));
        reg_src = suniv_dma_read(dma_dev, SUNIV_DDMA_SRC_ADR_REG(sdc->id));
        reg_dst = suniv_dma_read(dma_dev, SUNIV_DDMA_DES_ADR_REG(sdc->id));
        reg_byte = suniv_dma_read(dma_dev, SUNIV_DDMA_BYTE_CNT_REG(sdc->id));
    } else {
        reg_cfg = suniv_dma_read(dma_dev, SUNIV_NDMA_CFG_REG(sdc->id));
        reg_src = suniv_dma_read(dma_dev, SUNIV_NDMA_SRC_ADR_REG(sdc->id));
        reg_dst = suniv_dma_read(dma_dev, SUNIV_NDMA_DES_ADR_REG(sdc->id));
        reg_byte = suniv_dma_read(dma_dev, SUNIV_NDMA_BYTE_CNT_REG(sdc->id));
    }

    /* store diagnosis data */
    dma_dev->diag.reg.cfg      = reg_cfg;
    dma_dev->diag.reg.src_addr = reg_src;
    dma_dev->diag.reg.des_addr = reg_dst;
    dma_dev->diag.reg.byte_cnt = reg_byte;

    printk("======================== SUNIV DMA DUMP =======================\n");
    printk("\t\t\tid : %d type : %s", sdc->id, sdc->is_dedicated ? "DDMA" : "NDMA");
    printk("\t\t Interrupt control : 0x%x", reg_intc);
    printk("\t\t Interrupt pending : 0x%x", reg_intp);
    printk("\t\t Channel config : 0x%x", reg_cfg);
    printk("\t\t Source Address : 0x%x , %u", reg_src, reg_src);
    printk("\t\t Destnation Address : 0x%x , %u", reg_dst, reg_dst);
    printk("\t\t Byte count : 0x%x , %u", reg_byte, reg_byte);
    printk("======================== SUNIV DMA DUMP =======================\n");
}

static ssize_t suniv_dma_channel_num_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct suniv_dma *dma_dev = dev_get_drvdata(dev);
    struct suniv_dma_diagnosis *diag = &dma_dev->diag;
    int rc;

    rc = sprintf(buf, "%d", diag->cid);

    return rc;
}

static ssize_t suniv_dma_channel_num_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct suniv_dma *dma_dev = dev_get_drvdata(dev);
    struct suniv_dma_diagnosis *diag = &dma_dev->diag;
    u32 cid;
    int rc;

    mutex_lock(&dma_dev->mutex);

    rc = sscanf(buf, "%d", &cid);

    printk("%s, %d\n", __func__, cid);

    if (0 <= cid && cid < SUNIV_DMA_MAX_CHANNELS) {
        diag->cid = cid;
    } else {
        printk("%s, channel range should between 0 ~ 7\n", __func__);
        return -EINVAL;
    }

    mutex_unlock(&dma_dev->mutex);

    return count;
}
static DEVICE_ATTR(channel, S_IRUSR | S_IWUSR, suniv_dma_channel_num_show,
                   suniv_dma_channel_num_store);

static ssize_t suniv_dma_channel_reg_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct suniv_dma *dma_dev = dev_get_drvdata(dev);
    struct suniv_dma_diagnosis *diag = &dma_dev->diag;
    struct suniv_dma_chan *sdc = &(dma_dev->chan[diag->cid]);

    mutex_lock(&dma_dev->mutex);
    suniv_dma_reg_dump(sdc);
    mutex_unlock(&dma_dev->mutex);

    return 0;
}
// static ssize_t suniv_dma_channel_reg_store(struct device *dev,
//                                            struct device_attribute *attr,
//                                            const char *buf, size_t count)
// {
//     return 0;
// }
static DEVICE_ATTR(dump_register, S_IRUSR | S_IWUSR, suniv_dma_channel_reg_show,
                   NULL);

static struct attribute *suniv_dma_sysfs_attrs[] = {
    &dev_attr_channel.attr,
    &dev_attr_dump_register.attr,
    NULL,
};

static struct attribute_group suniv_dma_attribute_group = {
    .attrs = suniv_dma_sysfs_attrs,
};

static inline int suniv_dma_register_sysfs(struct device *dev,
                                           struct attribute_group *attr_group)
{
    return sysfs_create_group(&dev->kobj, attr_group);
}

static inline void suniv_dma_unregister_sysfs(struct device *dev,
                                              struct attribute_group *attr_group)
{
    sysfs_remove_group(&dev->kobj, attr_group);
}

/*
 * This routine is used to control the interrupt
 * of each hardware dma channel
 */
static void suniv_dma_set_interurpt(struct suniv_dma *dma_dev,
                                    struct suniv_dma_chan *chan,
                                    u32 end)
{
    u32 reg;
    unsigned long flags;

    printk("%s\n", __func__);
    spin_lock_irqsave(&dma_dev->lock, flags);

    reg = suniv_dma_read(dma_dev, SUNIV_DMA_INT_CTRL_REG);

    if (end && chan->is_dedicated)
        reg |= SUNIV_DDMA_FULL_TRANS_INT_EN(chan->id);
    else if (end && !chan->is_dedicated)
        reg |= SUNIV_NDMA_FULL_TRANS_INT_EN(chan->id);
    else if (!end && chan->is_dedicated)
        reg &= ~SUNIV_DDMA_FULL_TRANS_INT_EN(chan->id);
    else // (!end && !chan->is_dedicated)
        reg &= ~SUNIV_NDMA_FULL_TRANS_INT_EN(chan->id);

    suniv_dma_write(dma_dev, SUNIV_DMA_INT_CTRL_REG, reg);

    spin_unlock_irqrestore(&dma_dev->lock, flags);
}

static u32 suniv_dma_get_remain_bytes(struct suniv_dma_chan *sdc)
{
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(sdc);
    u32 remain_bytes;

    if (sdc->is_dedicated)
        remain_bytes = suniv_dma_read(dma_dev, SUNIV_DDMA_BYTE_CNT_REG(sdc->id));
    else
        remain_bytes = suniv_dma_read(dma_dev, SUNIV_NDMA_BYTE_CNT_REG(sdc->id));

    printk("%s, channel : %d  remain bytes : %d", __func__, sdc->id, remain_bytes);

    return remain_bytes;
}

static bool suniv_dma_check_trans_done(struct suniv_dma_chan *sdc)
{
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(sdc);
    u32 status;

    /*
     * According to the user manual, we should check the
     * bit 29 of config register of current dma channel
     * to know if DMA process is finished, but this bit
     * is used for setting if DMA in continuous mode.
     *
     * Otherwise, the bit 30 is used to check DMA channel
     * is busy, we do this check routine here.
     */
    if (sdc->is_dedicated)
        status = suniv_dma_read(dma_dev, SUNIV_DDMA_CFG_REG(sdc->id));
    else
        status = suniv_dma_read(dma_dev, SUNIV_NDMA_CFG_REG(sdc->id));

    if (status & SUNIV_DMA_BUSY)
        return false;

    return true;
}

/* The interrupt service routine of dma channel */
static irqreturn_t suniv_dma_isr(int irq, void *devid)
{
    struct suniv_dma *dma_dev = (struct suniv_dma *)devid;
    struct suniv_dma_chan *chan;
    unsigned long irq_pending;
    int bit;
    u32 mask;

    irq_pending = suniv_dma_read(dma_dev, SUNIV_DMA_INT_STA_REG);

    printk("%s, irq_pending: 0x%lx\n", __func__, irq_pending);
    for_each_set_bit(bit, &irq_pending, 32) {
        chan = &dma_dev->chan[bit >> 1];

        if (bit & 1) { /* full transfer interrupt */
            printk("%s, FULL transfer interrupt\n", __func__);
        } else {    /* half transfer interrupt */
            printk("%s, HALF transfer interrupt\n", __func__);
        }

        spin_lock(&dma_dev->lock);
        suniv_dma_set_interurpt(dma_dev, chan, 0);
        spin_unlock(&dma_dev->lock);

        mask =  chan->is_dedicated ? \
                SUNIV_DDMA_FULL_TRANS_INT_PENDING(chan->id) : \
                SUNIV_NDMA_FULL_TRANS_INT_PENDING(chan->id);

        /* set 1 to clear the interrupt pending */
        suniv_dma_update(dma_dev, SUNIV_DMA_INT_STA_REG, mask, mask);
    }

    return IRQ_HANDLED;
}

static void suniv_dma_set_chn_config(struct suniv_dma_chan *schan,
                                     struct suniv_dma_desc *sdesc)
{
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(schan);
    struct suniv_dma_chn_hw *hw = &sdesc->chan_hw;

    /* A bunch of `suniv_dma_write` here, config current dma channel */
    if (schan->is_dedicated) {
        printk("%s, Configuring DDMA chann: %d \n", __func__, schan->id);
        printk("%s, src : %u dst : %u  len : %d", __func__, hw->src_addr, hw->des_addr,
               hw->byte_cnt);
        suniv_dma_write(dma_dev, SUNIV_DDMA_SRC_ADR_REG(schan->id), hw->src_addr);
        suniv_dma_write(dma_dev, SUNIV_DDMA_DES_ADR_REG(schan->id), hw->des_addr);
        suniv_dma_write(dma_dev, SUNIV_DDMA_BYTE_CNT_REG(schan->id), hw->byte_cnt);
        suniv_dma_write(dma_dev, SUNIV_DDMA_CFG_REG(schan->id), hw->cfg);

    } else {
        printk("%s, Configuring NDMA chann: %d \n", __func__, schan->id);
        printk("%s, src : %u dst : %u  len : %d", __func__, hw->src_addr, hw->des_addr,
               hw->byte_cnt);
        suniv_dma_write(dma_dev, SUNIV_NDMA_SRC_ADR_REG(schan->id), hw->src_addr);
        suniv_dma_write(dma_dev, SUNIV_NDMA_DES_ADR_REG(schan->id), hw->des_addr);
        suniv_dma_write(dma_dev, SUNIV_NDMA_BYTE_CNT_REG(schan->id), hw->byte_cnt);
        suniv_dma_write(dma_dev, SUNIV_NDMA_CFG_REG(schan->id), hw->cfg);
    }

    /*
     * The DMA process should start soonly after filling the bit 31
     * "DMA Loading" of channel config register
     */
}

static void suniv_dma_start(struct suniv_dma_chan *chan)
{
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(chan);
    struct suniv_dma_prompt *prompt, *n;
    struct virt_dma_desc *vdesc;

    vdesc = vchan_next_desc(&chan->vchan);
    if (!vdesc)
        return;

    chan->desc = to_suniv_dma_desc(vdesc);

    if (likely(list_empty(&dma_dev->prompts.list))) {
        printk("%s, normal memcpy like process\n", __func__);
        /* Hardware Configuration */
        suniv_dma_set_interurpt(dma_dev, chan, 1);
        suniv_dma_set_chn_config(chan, chan->desc);
    } else {
        printk("%s, it's a scatter-gather process\n", __func__);
        /* it's a  scatter-gather process */
        list_for_each_entry_safe(prompt, n, &dma_dev->prompts.list, list) {
            memcpy(&chan->desc->chan_hw, &prompt->hw, sizeof(struct suniv_dma_chn_hw));

            suniv_dma_set_interurpt(dma_dev, chan, 1);
            suniv_dma_set_chn_config(chan, chan->desc);

            list_del(&prompt->list);
            kfree(prompt);
        }

    }

}

static int suniv_dma_config(struct dma_chan *chan,
                            struct dma_slave_config *config)
{
    struct suniv_dma_chan *sdc = to_suniv_dma_chan(chan);

    memcpy(&sdc->slave_cfg, config, sizeof(*config));

    return 0;
}

static struct dma_chan *suniv_dma_of_xlate(struct of_phandle_args *dma_spec,
                                           struct of_dma *ofdma)
{
    struct suniv_dma *dma_dev = ofdma->of_dma_data;
    struct suniv_dma_chan *sdc;
    struct dma_chan *chan = NULL;

    /*
     * In deivce tree, you should fill dmas cell like blow:
     *      dmas = <&dma 0x04>, <&dma 0x04>;
     *
     * Well, the 0x04 is the DRQ type needed by this node,
     * so we should store this num temporarily for configurating
     * the DMA channel.
     */
    u8 endpoint = dma_spec->args[1];

    chan = dma_get_any_slave_channel(&dma_dev->ddev);
    if (!chan)
        return NULL;

    sdc = to_suniv_dma_chan(chan);

    sdc->endpoint = endpoint;

    printk("%s, %s\n", __func__, dma_spec->np->name);
    return chan;
}

static int suniv_dma_alloc_chan_resources(struct dma_chan *chan)
{
    printk("%s\n", __func__);
    return 0;
}

static void suniv_dma_free_chan_reosuces(struct dma_chan *chan)
{
    printk("%s\n", __func__);
}

static int suniv_dma_terminate_all(struct dma_chan *chan)
{
    struct suniv_dma_chan *sdc = to_suniv_dma_chan(chan);
    struct suniv_dma *dma_dev = to_suniv_dma_by_dc(chan);
    printk("%s\n", __func__);

    if (sdc->is_dedicated)
        suniv_dma_write(dma_dev, SUNIV_DDMA_CFG_REG(sdc->id), ~SUNIV_DMA_LOADING);
    else
        suniv_dma_write(dma_dev, SUNIV_NDMA_CFG_REG(sdc->id), ~SUNIV_DMA_LOADING);

    return 0;
}

static enum dma_status suniv_dma_tx_status(struct dma_chan *chan,
                                           dma_cookie_t cookie,
                                           struct dma_tx_state *txstate)
{
    struct suniv_dma_chan *sdc = to_suniv_dma_chan(chan);
    struct suniv_dma *dma_dev = to_suniv_dma_by_dc(chan);
    // struct suniv_dma_chn_hw *hw;
    // struct suniv_dma_desc *desc;
    // struct virt_dma_desc *vd;
    unsigned long flags;
    u32 pos = 0;
    int rc = 0;

    spin_lock_irqsave(&dma_dev->lock, flags);
    printk("%s, is being called\n", __func__);
    rc = dma_cookie_status(chan, cookie, txstate);
    if (rc == DMA_COMPLETE || !txstate)
        return rc;
    // vd = vchan_find_desc(&sdc->vchan, cookie);
    // if (!vd)
    //     goto exit;

    sdc->trans_done = suniv_dma_check_trans_done(sdc);
    printk("%s, check transmit state : %s", __func__, sdc->trans_done ? "idle" : "busy");
    if (sdc->trans_done == true) {
        vchan_cookie_complete(&sdc->desc->vdesc);
        rc = DMA_COMPLETE;
    } else {
        rc = DMA_IN_PROGRESS;
    }

    sdc->desc->remain_bytes = suniv_dma_get_remain_bytes(sdc);

    pos = sdc->desc->remain_bytes;

    // exit:
    dma_set_residue(txstate, pos);

    spin_unlock_irqrestore(&dma_dev->lock, flags);

    return rc;
}

static void suniv_dma_issue_pending(struct dma_chan *c)
{
    struct suniv_dma_chan *chan = to_suniv_dma_chan(c);
    unsigned long flags;

    spin_lock_irqsave(&chan->vchan.lock, flags);
    printk("%s\n", __func__);
    if (vchan_issue_pending(&chan->vchan) && chan->desc)
        suniv_dma_start(chan);
    spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static struct dma_async_tx_descriptor *
suniv_dma_prep_dma_memcpy(struct dma_chan *c, dma_addr_t dst, dma_addr_t src,
                          size_t len, unsigned long flags)
{
    struct suniv_dma_chn_hw *hw;
    struct suniv_dma_desc *desc;
    struct suniv_dma_chan *chan = to_suniv_dma_chan(c);

    printk("%s, src: %u, dst: %u, len: %d", __func__, src, dst, len);

    desc = kzalloc(sizeof(struct suniv_dma_desc), GFP_NOWAIT);
    if (!desc)
        return NULL;

    chan->desc = desc;

    desc->remain_bytes = 0;

    /* Save hardware params */
    hw = &desc->chan_hw;

    hw->des_addr = dst;
    hw->src_addr = src;

    /* check the format of data */
    if (IS_ALIGNED(len, 4)) {
        hw->datawidth = SUNIV_DMA_4_BYTE;
        hw->brust_len = SUNIV_DMA_BURST_4;
    } else if (IS_ALIGNED(len, 2)) {
        hw->datawidth = SUNIV_DMA_2_BYTE;
        hw->brust_len = SUNIV_DMA_BURST_1;
    } else {
        hw->datawidth = SUNIV_DMA_1_BYTE;
        hw->brust_len = SUNIV_DMA_BURST_1;
    }

    if (chan->is_dedicated) {
        hw->byte_cnt = len & SUNIV_DDMA_BYET_CNT_MASK;

        hw->cfg |= SUNIV_DDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_DES_DRQ_TYPE_OFFSET;
        hw->cfg |= SUNIV_DDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_SRC_DRQ_TYPE_OFFSET;
        hw->cfg |= SUNIV_DMA_SRC_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_LINER);
        hw->cfg |= SUNIV_DMA_DES_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_LINER);
    } else {
        hw->byte_cnt = len & SUNIV_NDMA_BYET_CNT_MASK;

        hw->cfg |= SUNIV_NDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_DES_DRQ_TYPE_OFFSET;
        hw->cfg |= SUNIV_NDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_SRC_DRQ_TYPE_OFFSET;
        hw->cfg |= SUNIV_DMA_SRC_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_IO);
        hw->cfg |= SUNIV_DMA_DES_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_IO);
    }

    /* Common configuration */
    hw->cfg |= SUNIV_DMA_SRC_DATAWIDTH(hw->datawidth);
    hw->cfg |= SUNIV_DMA_SRC_BRUST_LEN(hw->brust_len);

    hw->cfg |= SUNIV_DMA_DES_DATAWIDTH(hw->datawidth);
    hw->cfg |= SUNIV_DMA_DES_BRUST_LEN(hw->brust_len);

    hw->cfg |= SUNIV_DMA_REMAIN_BYTE_CNT_READ_EN;
    hw->cfg |= SUNIV_DMA_LOADING;

    return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *
suniv_dma_prep_slave_sg(struct dma_chan *chan,
                        struct scatterlist *sgl,
                        unsigned int sg_len, enum dma_transfer_direction direction,
                        unsigned long flags, void *context)
{
    struct suniv_dma_chan *sdc = to_suniv_dma_chan(chan);
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(sdc);
    struct dma_slave_config *slave_cfg = &sdc->slave_cfg;
    struct suniv_dma_prompt *prompt;
    struct suniv_dma_desc    *desc;
    struct suniv_dma_chn_hw   *hw;
    struct scatterlist *sg;
    u32 len;
    int i;

    if (!sgl)
        return NULL;

    if (!is_slave_direction(direction)) {
        dev_err(chan2dev(chan), "Invalid DMA direction\n");
        return NULL;
    }

    desc = kzalloc(sizeof(struct suniv_dma_desc), GFP_NOWAIT);
    if (!desc)
        return NULL;

    /* Huh, always forget save this into my channel */
    sdc->desc = desc;

    desc->dir = direction;
    hw = &desc->chan_hw;

    if (direction == DMA_MEM_TO_DEV) {
        hw->cfg |= SUNIV_DMA_DES_DRQ_TYPE(sdc->endpoint);
        hw->cfg |= SUNIV_DMA_DES_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_IO);

        hw->cfg |= SUNIV_DMA_SRC_DRQ_TYPE(sdc->is_dedicated ?
                                          SUNIV_DDMA_DRQ_TYPE_SDRAM : SUNIV_NDMA_DRQ_TYPE_SDRAM);
        hw->cfg |= SUNIV_DMA_SRC_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_LINER);
    } else {
        hw->cfg |= SUNIV_DMA_DES_DRQ_TYPE(sdc->is_dedicated ?
                                          SUNIV_DDMA_DRQ_TYPE_SDRAM : SUNIV_NDMA_DRQ_TYPE_SDRAM);
        hw->cfg |= SUNIV_DMA_DES_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_LINER);

        hw->cfg |= SUNIV_DMA_SRC_DRQ_TYPE(sdc->endpoint);
        hw->cfg |= SUNIV_DMA_SRC_ADDR_TYPE(SUNIV_DMA_ADDR_TYPE_IO);
    }

    for_each_sg(sgl, sg, sg_len, i) {
        printk("%s, prompt : %d preparing ...\n", __func__, i);
        prompt = kzalloc(sizeof(*prompt), GFP_KERNEL);

        len = sg_dma_len(sg);

        if (IS_ALIGNED(len, 4))
            hw->brust_len = SUNIV_DMA_BURST_4;

        if (sdc->is_dedicated)
            hw->byte_cnt = len & SUNIV_DDMA_BYET_CNT_MASK;
        else
            hw->byte_cnt = len & SUNIV_NDMA_BYET_CNT_MASK;

        if (direction == DMA_MEM_TO_DEV) {
            hw->datawidth = slave_cfg->src_addr_width;
            hw->src_addr = sg_dma_address(sg);
            hw->des_addr = slave_cfg->dst_addr;
        } else {
            hw->datawidth = slave_cfg->dst_addr_width;
            hw->src_addr = slave_cfg->src_addr;
            hw->des_addr = sg_dma_address(sg);
        }

        /* Common configuration */
        hw->cfg |= SUNIV_DMA_SRC_DATAWIDTH(hw->datawidth);
        hw->cfg |= SUNIV_DMA_SRC_BRUST_LEN(hw->brust_len);

        hw->cfg |= SUNIV_DMA_DES_DATAWIDTH(hw->datawidth);
        hw->cfg |= SUNIV_DMA_DES_BRUST_LEN(hw->brust_len);

        hw->cfg |= SUNIV_DMA_REMAIN_BYTE_CNT_READ_EN;
        hw->cfg |= SUNIV_DMA_LOADING;

        memcpy(&prompt->hw, hw, sizeof(struct suniv_dma_chn_hw));

        list_add_tail(&prompt->list, &dma_dev->prompts.list);
    }

    return vchan_tx_prep(&sdc->vchan, &desc->vdesc, flags);
}

static void suniv_dma_desc_free(struct virt_dma_desc *vdesc)
{
    kfree(container_of(vdesc, struct suniv_dma_desc, vdesc));
}

static int suniv_dma_probe(struct platform_device *pdev)
{
    int i, j, rc;
    struct suniv_dma *dma_dev;
    struct dma_device *dd;
    struct suniv_dma_chan *chan;

    printk("%s\n", __func__);

    printk("%s, alloc memory of dma_dev\n", __func__);
    dma_dev = devm_kzalloc(&pdev->dev, sizeof(struct suniv_dma),
                           GFP_KERNEL);

    if (!dma_dev)
        return -ENOMEM;

    dd = &dma_dev->ddev;

    printk("%s, ioremap the base addr of DMA registers\n", __func__);
    dma_dev->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(dma_dev->base))
        return PTR_ERR(dma_dev->base);

    printk("%s, DMA register base: %p\n", __func__, dma_dev->base);

    dma_dev->irq = platform_get_irq(pdev, 0);
    if (dma_dev->irq < 0) {
        printk("%s, can't get irq\n", __func__);
        return dma_dev->irq;
    }
    printk("%s, Got irq number %d from device\n", __func__, dma_dev->irq);


    rc = devm_request_irq(&pdev->dev, dma_dev->irq,
                          suniv_dma_isr, 0,
                          DRV_NAME " device", dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "request irq failed with err %d\n", rc);
        return rc;
    }

    /* Initialize locks */
    spin_lock_init(&dma_dev->lock);
    mutex_init(&dma_dev->mutex);

    /* Acquire clocks */
    dma_dev->hclk = devm_clk_get(&pdev->dev, "ahb");
    if (IS_ERR(dma_dev->hclk)) {
        dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
        return PTR_ERR(dma_dev->hclk);
    }

    dma_dev->mclk = devm_clk_get(&pdev->dev, "mod");
    if (IS_ERR(dma_dev->mclk)) {
        dev_err(&pdev->dev, "Unable to acquire module clock\n");
        return PTR_ERR(dma_dev->mclk);
    }

    dma_dev->rstc = devm_reset_control_array_get_exclusive(&pdev->dev);
    if (IS_ERR(dma_dev->rstc)) {
        dev_err(&pdev->dev, "Unable to get reset controller\n");
        return PTR_ERR(dma_dev->rstc);
    }

    dma_cap_set(DMA_MEMCPY, dd->cap_mask);
    dma_cap_set(DMA_SLAVE, dd->cap_mask);
    dd->device_alloc_chan_resources = suniv_dma_alloc_chan_resources;
    dd->device_free_chan_resources  = suniv_dma_free_chan_reosuces;
    dd->device_terminate_all        = suniv_dma_terminate_all;
    dd->device_config               = suniv_dma_config;
    dd->device_tx_status            = suniv_dma_tx_status;
    dd->device_issue_pending        = suniv_dma_issue_pending;
    dd->device_prep_dma_memcpy      = suniv_dma_prep_dma_memcpy;
    dd->device_prep_slave_sg        = suniv_dma_prep_slave_sg;

    dd->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
                          BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
                          BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
    dd->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
                          BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
                          BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
    dd->directions = BIT(DMA_MEM_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
                     BIT(DMA_DEV_TO_MEM);
    dd->max_burst = SUNIV_DMA_MAX_BURST;
    dd->dev = &pdev->dev;
    INIT_LIST_HEAD(&dd->channels);
    INIT_LIST_HEAD(&dma_dev->prompts.list);

    for (i = 0; i < SUNIV_DMA_NDMA_NR_CHANNELS; i++) {
        chan = &dma_dev->chan[i];
        chan->id = i;
        chan->vchan.desc_free = suniv_dma_desc_free;
        chan->is_dedicated = false;
        vchan_init(&chan->vchan, dd);
    }

    for (j = 0; i < SUNIV_DMA_MAX_CHANNELS; j++, i++) {
        chan = &dma_dev->chan[i];
        chan->id = j;
        chan->vchan.desc_free = suniv_dma_desc_free;
        chan->is_dedicated = true;
        vchan_init(&chan->vchan, dd);
    }

    /* set private data */
    platform_set_drvdata(pdev, dma_dev);
    dev_set_drvdata(&pdev->dev, dma_dev);

    /* enable clks and reset dma controller */
    rc = suniv_dma_enable(dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "enable suniv dma failed:%d", rc);
        return rc;
    }
    pr_debug("%s, suniv dma is enabled\n", __func__);

    rc = dma_async_device_register(dd);
    if (rc) {
        dev_err(&pdev->dev, "register dma device failed:%d", rc);

    }

    rc = of_dma_controller_register(pdev->dev.of_node,
                                    suniv_dma_of_xlate, dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "suniv DMA OF registration failed:%d\n", rc);
        goto err_dma_dev_unregister;
    }

    rc = suniv_dma_register_sysfs(&pdev->dev, &suniv_dma_attribute_group);
    if (rc) {
        goto err_of_dma_unregister;
    }

    return 0;

err_of_dma_unregister:
    of_dma_controller_free(pdev->dev.of_node);

err_dma_dev_unregister:
    dma_async_device_unregister(dd);

    return rc;
}

static int suniv_dma_remove(struct platform_device *pdev)
{
    struct suniv_dma *dma_dev = platform_get_drvdata(pdev);

    printk("%s\n", __func__);
    if (dma_dev->irq > 0)
        devm_free_irq(&pdev->dev, dma_dev->irq, dma_dev);

    /* channels clean */
    suniv_dma_unregister_sysfs(&pdev->dev, &suniv_dma_attribute_group);
    of_dma_controller_free(pdev->dev.of_node);
    dma_async_device_unregister(&dma_dev->ddev);
    suniv_dma_disable(dma_dev);

    return 0;
}

static struct of_device_id suniv_dma_of_match_table[] = {
    {.compatible = "allwinner,suniv-f1c100s-dma" },
    { /* KEEP THIS */ }
};
MODULE_DEVICE_TABLE(of, suniv_dma_of_match_table);

static struct platform_driver suniv_dma_driver = {
    .probe  = suniv_dma_probe,
    .remove = suniv_dma_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(suniv_dma_of_match_table),
    },
};

module_platform_driver(suniv_dma_driver);

MODULE_AUTHOR("IotaHydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("A simple DMA Engine driver for Allwinner F series SoC");
MODULE_LICENSE("GPL");
