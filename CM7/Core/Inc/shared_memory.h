#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Definicja długości komunikatu przechowywanego we współdzielonej pamięci */
#define SHARED_MESSAGE_SIZE 32

/**
  * @brief  Struktura definiująca obszar współdzielonej pamięci.
  */
typedef struct
{
    char message[SHARED_MESSAGE_SIZE];  /**< Bufor przechowujący komunikat */
    /* Dodatkowe pola mogą być dodane w przyszłości */
} SharedMemory_TypeDef;

/* Deklaracja zmiennej globalnej współdzielonej pamięci */
extern SharedMemory_TypeDef *pSharedMem;

/**
  * @brief  Funkcja inicjalizująca moduł współdzielonej pamięci.
  * @note   W tym przykładzie inicjalizacja odbywa się poprzez przypisanie
  *         adresu statycznie zdefiniowanego obszaru pamięci, który można umieścić
  *         w dedykowanej sekcji.
  * @retval Wskaźnik do struktury SharedMemory_TypeDef.
  */
SharedMemory_TypeDef *SharedMemory_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_MEMORY_H */
