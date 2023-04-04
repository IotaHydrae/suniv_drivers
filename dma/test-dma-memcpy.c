#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>


struct dma_chan *dma_chan;
dma_cookie_t cookie;

void *dst_addr, *src_addr;
dma_addr_t phy_dst, phy_src;

static int __init test_dma_init(void)
{
    return 0;
}

static void __exit test_dma_exit(void)
{
}

module_init(test_dma_init);
module_exit(test_dma_exit);
MODULE_AUTHOR("IotaHydrae writeforever@foxmail.com");
MODULE_LICENSE("GPL");
