#include "shared_memory.h"
#include <string.h>

/* Opcjonalnie: Atrybut umieszczenia w specjalnej sekcji pamięci współdzielonej.
   Upewnij się, że linker script jest odpowiednio skonfigurowany, aby ta sekcja była widoczna dla obu rdzeni. */
#if defined (__GNUC__)
SharedMemory_TypeDef sharedMemoryInstance __attribute__((section(".shared_memory"))) = {0};
#else
SharedMemory_TypeDef sharedMemoryInstance = {0};
#endif

/* Definicja wskaźnika globalnego */
SharedMemory_TypeDef *pSharedMem = &sharedMemoryInstance;
extern SharedMemory_TypeDef *pSharedMem;
SharedMemory_TypeDef *SharedMemory_Init(void)
{
    /* Inicjalizacja bufora – opcjonalnie, można dodać dodatkowe procedury inicjalizacyjne */
    memset(sharedMemoryInstance.message, 0, SHARED_MESSAGE_SIZE);
    return pSharedMem;
}
