/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// #include <libopencm3/stm32/flash.h>
// #include <libopencm3/stm32/crc.h>
#include "hal.h"
#include "params.h"
#include "param_save.h"
#include "hwdefs.h"
#include "my_string.h"

#define NUM_PARAMS ((PARAM_BLKSIZE - 8) / sizeof(PARAM_ENTRY))
#define PARAM_WORDS (PARAM_BLKSIZE / 4)

#define FlashWaitWhileBusy() { while (FLASH->SR & FLASH_SR_BSY) {} }
#define FlashLock() { FLASH->CR |= FLASH_CR_LOCK; }

uint32_t gCrc = 0;

void crc_callback(CRCDriver *crcp, uint32_t crc) {
  (void)crcp;
  gCrc = crc;
}

/*
 * CRC32 configuration
 */
static const CRCConfig crc32_config = {
  .poly_size         = 32,
  .poly              = 0x04C11DB7,
  .initial_val       = 0xFFFFFFFF,
  .final_val         = 0xFFFFFFFF,
  .reflect_data      = 1,
  .reflect_remainder = 1
};

#if CRC_USE_DMA == TRUE
/*
 * CRC32 configuration
 */
static const CRCConfig crc32_dma_config = {
  .poly_size         = 32,
  .poly              = 0x04C11DB7,
  .initial_val       = 0xFFFFFFFF,
  .final_val         = 0xFFFFFFFF,
  .reflect_data      = 1,
  .reflect_remainder = 1,
  .end_cb = crc_callback
};
#endif

uint32_t crc_calculate(uint32_t data)
{
	CRC->DR = data;
	return CRC->DR;
}

uint32_t crc_calculate_block(uint32_t *datap, int size)
{
	int i;

	for (i = 0; i < size; i++) {
		CRC->DR = datap[i];
	}

	return CRC->DR;
}

/**
 * @brief Unlock the flash memory for write access.
 * @return HAL_SUCCESS  Unlock was successful.
 * @return HAL_FAILED    Unlock failed.
 */
static bool FlashUnlock(void) {
	/* Check if unlock is really needed */
	if (!(FLASH->CR  & FLASH_CR_LOCK))
		return HAL_SUCCESS;

	/* Write magic unlock sequence */
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	/* Check if unlock was successful */
	if (FLASH->CR  & FLASH_CR_LOCK)
		return HAL_FAILED;
	return HAL_SUCCESS;
}

int FlashErasePage(uintptr_t page_address) {

   FlashWaitWhileBusy();

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = page_address;
	FLASH->CR |= FLASH_CR_STRT;

	FlashWaitWhileBusy();

	FLASH->CR  &= ~FLASH_CR_PER;

	return HAL_SUCCESS;
}

typedef struct
{
   uint16_t key;
   uint8_t dummy;
   uint8_t flags;
   uint32_t value;
} PARAM_ENTRY;

typedef struct
{
   PARAM_ENTRY data[NUM_PARAMS];
   uint32_t crc;
   uint32_t padding;
} PARAM_PAGE;

/**
* Save parameters to flash
*
* @return CRC of parameter flash page
*/
uint32_t parm_save()
{
   PARAM_PAGE parmPage;
   unsigned int idx;

   // crc_reset();
   crcAcquireUnit(&CRCD1);
   crcReset(&CRCD1);
   memset32((int*)&parmPage, 0xFFFFFFFF, PARAM_WORDS);

   //Copy parameter values and keys to block structure
   for (idx = 0; Param::IsParam((Param::PARAM_NUM)idx) && idx < NUM_PARAMS; idx++)
   {
      const Param::Attributes *pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      parmPage.data[idx].flags = (uint8_t)Param::GetFlag((Param::PARAM_NUM)idx);
      parmPage.data[idx].key = pAtr->id;
      parmPage.data[idx].value = Param::Get((Param::PARAM_NUM)idx);
   }

   parmPage.crc = crc_calculate_block(((uint32_t*)&parmPage), (2 * NUM_PARAMS));
   // parmPage.crc = crcCalc(&CRCD1, sizeof(parmPage), &parmPage);
   FlashUnlock();
   FlashErasePage(PARAM_ADDRESS);

   for (idx = 0; idx < PARAM_WORDS; idx++)
   {
      // uint32_t* pData = ((uint32_t*)&parmPage) + idx;
      // flash_program_word(PARAM_ADDRESS + idx * sizeof(uint32_t), *pData);
   }
   FlashLock();
   return parmPage.crc;
   return 1;
}

/**
* Load parameters from flash
*
* @retval 0 Parameters loaded successfully
* @retval -1 CRC error, parameters not loaded
*/
int parm_load()
{
   PARAM_PAGE *parmPage = (PARAM_PAGE *)PARAM_ADDRESS;

   // crcAcquireUnit(&CRCD1);
   // crcStart(&CRCD1, &crc32_config);
   // crcReset(&CRCD1);

   // uint32_t crc = crcCalc(&CRCD1, sizeof(parmPage), &parmPage);
   uint32_t crc = crc_calculate_block(((uint32_t*)parmPage), (2 * NUM_PARAMS)); // TODO use ChibiOS CRC

   // crcStop(&CRCD1);
   // crcReleaseUnit(&CRCD1);

   if (crc == parmPage->crc)
   {
      for (unsigned int idxPage = 0; idxPage < NUM_PARAMS; idxPage++)
      {
         Param::PARAM_NUM idx = Param::NumFromId(parmPage->data[idxPage].key);
         if (idx != Param::PARAM_INVALID && parmPage->data[idxPage].key > 0)
         {
            Param::SetFlt(idx, parmPage->data[idxPage].value);
            Param::SetFlagsRaw(idx, parmPage->data[idxPage].flags);
         }
      }
      return 0;
   }

   return -1;
}
