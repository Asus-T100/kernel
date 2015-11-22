#include <linux/kernel.h>
#include <linux/random.h>

/* Generate UUID by invoking kernel library */
void generate_key(char *buffer)
{
    char sysctl_bootid[16];

    generate_random_uuid(sysctl_bootid);
    sprintf(buffer, "%pU", sysctl_bootid);
}

