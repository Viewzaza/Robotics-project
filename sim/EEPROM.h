#pragma once
// Mock AVR EEPROM for the host simulator: 1 KB, backed by RAM, optionally
// persisted to a file so crash-resume can be exercised.
#include <cstring>
#include <cstdio>

class EEPROMClass {
public:
  unsigned char data[1024];
  EEPROMClass() { memset(data, 0xFF, sizeof data); }
  unsigned char read(int a) { return (a >= 0 && a < 1024) ? data[a] : 0xFF; }
  void write(int a, unsigned char v) { if (a >= 0 && a < 1024) data[a] = v; }
  void update(int a, unsigned char v) { if (read(a) != v) write(a, v); }
  template <class T> T& get(int a, T& t) {
    if (a >= 0 && a + (int)sizeof(T) <= 1024) memcpy(&t, data + a, sizeof(T));
    return t;
  }
  template <class T> const T& put(int a, const T& t) {
    if (a >= 0 && a + (int)sizeof(T) <= 1024) memcpy(data + a, &t, sizeof(T));
    return t;
  }
  void load(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return;
    fread(data, 1, sizeof data, f);
    fclose(f);
  }
  void save(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fwrite(data, 1, sizeof data, f);
    fclose(f);
  }
};
extern EEPROMClass EEPROM;
