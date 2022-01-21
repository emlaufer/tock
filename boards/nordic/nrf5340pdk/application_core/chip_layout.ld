/* Memory Space Definitions, 1M flash, 512K ram */
MEMORY
{
  rom (rx)  : ORIGIN = 0x00000000, LENGTH = 192K
  prog (rx) : ORIGIN = 0x00030000, LENGTH = 832K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

MPU_MIN_ALIGN = 8K;

SEC_FLASH_REG_SIZE = 0x4000; /* 16 kib */
/* technically this can change sizes, but for ease of linking we will set this to the max value */
/* helps linking because it must be placed at the END of a secure region... */
/* TODO: make this scale based on the nsc size... */
SEC_FLASH_NSC_SIZE = 0x200; 
