#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

int main(int argc, char **argv)
{
    // int len = 14951;
    int len = 1 << 20;
    FILE *fp;
    struct timeval tv_start, tv_end;

    unsigned char *src = malloc(len);
    unsigned char *dst = malloc(len);

    fp = fopen("/dev/urandom", "r");
    fread(src, 1, len, fp);
    fclose(fp);

    gettimeofday(&tv_start, NULL);
    memcpy(dst, src, len);
    gettimeofday(&tv_end, NULL);

    int rc = memcmp(src, dst, len);
    if (rc) {
        printf("check failed\n");
    }

    int cost = tv_end.tv_sec * 1000 - tv_start.tv_sec * 1000;
    printf("%d\n", cost);
    if (cost == 0)
        cost = ((tv_end.tv_usec) - (tv_start.tv_usec));
    else
        cost += (tv_end.tv_usec / 10^3 - tv_start.tv_usec / 10^3);
    printf("cost : %d ms\n", cost);
    printf("%d B/ms\n", len/cost);
    printf("%.3f KB/s\n", len/cost*1000.f);
    printf("%d M/s\n", len/cost);

    return 0;
}
