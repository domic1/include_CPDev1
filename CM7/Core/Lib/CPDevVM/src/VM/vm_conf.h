#ifndef VM_CONF_H
#define VM_CONF_H

//--------------------------------------------------------
// WERSJA MASZYNY CPDev
//--------------------------------------------------------
#define VM_M16B      // lub VM_M32B jeżeli korzystasz z 32b programów
//#define VM_M32B
//#define VM_M8B
//#define VM_M32BA

//--------------------------------------------------------
// Platforma i CPU
//--------------------------------------------------------
#define VM_STM16B
#define VM_ARM

//--------------------------------------------------------
// CPDev VM - użycie ticka z CubeHAL
//--------------------------------------------------------
#define VM_TICK_FUNC HAL_GetTick

//--------------------------------------------------------
// Optymalizacja - wyłącz nieużywane funkcje
//--------------------------------------------------------
#define VM_DISABLE_FS     // brak filesystemu
#define VM_DISABLE_EEPROM // brak eepromu

//--------------------------------------------------------
// Debug przez UART (jeśli chcesz logi CPDev na UART2)
//--------------------------------------------------------
//#define VM_DEBUG_UART

//--------------------------------------------------------
// RTC - jeśli korzystasz z RTC w CPDev
//--------------------------------------------------------
#define VM_USE_RTC

//--------------------------------------------------------
// RAM Section - jeśli masz własną sekcję RAM w linkerze
//--------------------------------------------------------
//#define VM_RAM_SECTION

#endif // VM_CONF_H
