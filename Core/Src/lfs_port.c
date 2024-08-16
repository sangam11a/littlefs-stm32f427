
#ifndef __LFS_PORT_H
#define __LFS_PORT_H

//#include "MT25Q.h"  // Include the MT25QL header file
#include "lfs.h"

#define MT25QL_SECTOR_SIZE 4096
#define MT25QL_SECTOR_NUM 1024  // Total size: 4096 * 1024 = 4M Byte

int lfs_spi_flash_init(struct lfs_config *cfg);
int lfs_spi_flash_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int lfs_spi_flash_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int lfs_spi_flash_erase(const struct lfs_config *cfg, lfs_block_t block);
int lfs_spi_flash_sync(const struct lfs_config *cfg);

int lfs_spi_flash_init(struct lfs_config *cfg) {
    MT25Q_Init();  // Initialize MT25QL flash
    MT25Q_Reset();  // Reset the MT25QL flash to its default state
    if (MT25Q_ReadID() == 0x9e) {
        cfg->read = lfs_spi_flash_read;
        cfg->prog = lfs_spi_flash_prog;
        cfg->erase = lfs_spi_flash_erase;
        cfg->sync = lfs_spi_flash_sync;

        cfg->read_size = 16;
        cfg->prog_size = 16;
        cfg->block_size = MT25QL_SECTOR_SIZE;
        cfg->block_count = MT25QL_SECTOR_NUM;
        cfg->block_cycles = 500;
        cfg->cache_size = 16;
        cfg->lookahead_size = 16;
        return LFS_ERR_OK;
    } else {
        return LFS_ERR_IO;
    }
}

int lfs_spi_flash_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    LFS_ASSERT(off % cfg->read_size == 0);
    LFS_ASSERT(size % cfg->read_size == 0);
    LFS_ASSERT(block < cfg->block_count);
    MT25Q_ReadData(block * cfg->block_size + off, (uint8_t *)buffer, size);
    return LFS_ERR_OK;
}

int lfs_spi_flash_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    LFS_ASSERT(off % cfg->prog_size == 0);
    LFS_ASSERT(size % cfg->prog_size == 0);
    LFS_ASSERT(block < cfg->block_count);
    MT25Q_WriteEnable();
    MT25Q_Program(block * cfg->block_size + off, (uint8_t *)buffer, size);
    return LFS_ERR_OK;
}

int lfs_spi_flash_erase(const struct lfs_config *cfg, lfs_block_t block) {
    LFS_ASSERT(block < cfg->block_count);
    MT25Q_WriteEnable();
    MT25Q_SectorErase(block * cfg->block_size);
    return LFS_ERR_OK;
}

int lfs_spi_flash_sync(const struct lfs_config *cfg) {
    return LFS_ERR_OK;
}

#endif // __LFS_PORT_H
