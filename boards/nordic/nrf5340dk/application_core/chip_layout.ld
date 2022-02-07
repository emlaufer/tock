/* Memory Space Definitions, 1M flash, 512K ram */
MEMORY
{
  /* TODO: it would be nice to automatically lay out rom into srom and nsrom
   *       based on used space */
  srom(rx)    : ORIGIN = 0x00000000, LENGTH = 384K
  nsrom(rx)   : ORIGIN = 0x00030000, LENGTH = 128K
  prog (rx)   : ORIGIN = 0x00050000, LENGTH = 512K
  sram (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
  nsram (rwx) : ORIGIN = 0x20020000, LENGTH = 384K
}

MPU_MIN_ALIGN = 8K;

SEC_FLASH_REG_SIZE = 0x4000; /* 16 kib */
ASSERT(ORIGIN(srom) % SEC_FLASH_REG_SIZE == 0, "SROM must begin on a secure region boundary!");
ASSERT((ORIGIN(srom) + LENGTH(srom)) % SEC_FLASH_REG_SIZE == 0, "SROM must end on a secure region boundary!");
SEC_RAM_REG_SIZE = 0x2000; /* 8 KiB */
ASSERT(ORIGIN(sram) % SEC_RAM_REG_SIZE == 0, "SRAM must begin on a secure region boundary!");
ASSERT((ORIGIN(sram) + LENGTH(sram)) % SEC_RAM_REG_SIZE == 0, "SRAM must begin on a secure region boundary!");

/* technically this can change sizes, but for ease of linking we will set this to the max value */
/* helps linking because it must be placed at the END of a secure region... */
/* TODO: make this scale based on the nsc size... */
SEC_FLASH_NSC_SIZE = 0x200;
