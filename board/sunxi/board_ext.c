#include <linux/compiler.h>
#include <mmc.h>
#include <private_uboot.h>
#include <serial.h>
#include <spare_head.h>

DECLARE_GLOBAL_DATA_PTR;

void set_boot_work_mode(int work_mode)
{
//       uboot_spare_head.boot_data.work_mode = work_mode;
}

int get_boot_work_mode(void)
{
//	return uboot_spare_head.boot_data.work_mode;
	return 0;
}

int sunxi_probe_securemode(void)
{
	return 0;
}

int dram_init(void)
{
    uint dram_size = 128;

    dram_size = dram_size > 2048 ? 2048 : dram_size;

    if(dram_size)
        gd->ram_size = dram_size * 1024 * 1024;
    else
        gd->ram_size = get_ram_size((long *)PHYS_SDRAM_0, PHYS_SDRAM_0_SIZE);

    return 0;
}

int board_late_init(void)
{
    return 0;
}

int sunxi_set_sramc_mode(void)
{
    return 0;
}
