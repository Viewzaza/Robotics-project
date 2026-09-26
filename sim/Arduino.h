// Minimal Arduino API mock for host-side simulation (g++).
#pragma once
#include <string>
#include <cstdint>
#include <cmath>
#include <cstdio>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

// On AVR F() puts the literal in flash and returns a __FlashStringHelper*.
// On the host both collapse to a plain C string.
typedef char __FlashStringHelper;
#define F(x) ((const __FlashStringHelper*)(x))

// Analog pin numbers, matching the real Nano mapping.
enum { A0 = 14, A1, A2, A3, A4, A5, A6, A7 };

// ---- Arduino String, backed by std::string ----
class String {
public:
  std::string s;
  String() {}
  String(const char* c) : s(c) {}
  String(const std::string& c) : s(c) {}
  String(int v) { char b[16]; snprintf(b, sizeof b, "%d", v); s = b; }
  String& operator+=(const char* c) { s += c; return *this; }
  String& operator+=(const String& o) { s += o.s; return *this; }
  bool operator==(const char* c) const { return s == c; }
  bool operator==(const String& o) const { return s == o.s; }
  bool operator!=(const char* c) const { return s != c; }
  const char* c_str() const { return s.c_str(); }
  unsigned length() const { return (unsigned)s.size(); }
  char charAt(unsigned i) const { return s[i]; }
};
inline String operator+(const String& a, const String& b) { return String(a.s + b.s); }
inline String operator+(const String& a, const char* b) { return String(a.s + b); }

// ---- time ----
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned long us);

// ---- io ----
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int  digitalRead(int pin);
void analogWrite(int pin, int value);
int  analogRead(int pin);

// ---- math helpers ----
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
template <class T> inline T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- Serial ----
struct SerialClass {
  bool echo = false;
  void begin(long) {}
  void print(const String& v)   { if (echo) printf("%s", v.c_str()); }
  void print(const char* v)     { if (echo) printf("%s", v); }
  void print(char v)            { if (echo) printf("%c", v); }   /* Arduino has this; without it ' ' prints as 32 */
  void print(int v)             { if (echo) printf("%d", v); }
  void print(long v)            { if (echo) printf("%ld", v); }
  void print(double v)          { if (echo) printf("%g", v); }
  void println()                { if (echo) printf("\n"); }
  void println(const String& v) { if (echo) printf("%s\n", v.c_str()); }
  void println(const char* v)   { if (echo) printf("%s\n", v); }
  void println(int v)           { if (echo) printf("%d\n", v); }
  void println(long v)          { if (echo) printf("%ld\n", v); }
  void println(double v)        { if (echo) printf("%g\n", v); }

  /* Real-Arduino overloads that the one-argument set above cannot express.
   * Every one of these calls was a compile error before (unsigned arguments
   * were ambiguous, two-argument forms did not exist), so adding them changes
   * no existing output -- they only let calibration / diagnostic code that
   * builds for the Nano also build here. */
  void print(unsigned int v)            { if (echo) printf("%u", v); }
  void print(unsigned long v)           { if (echo) printf("%lu", v); }
  void println(unsigned int v)          { if (echo) printf("%u\n", v); }
  void println(unsigned long v)         { if (echo) printf("%lu\n", v); }
  void print(long v, int base)          { if (echo) printBase(v, base); }
  void print(int v, int base)           { print((long)v, base); }
  void print(unsigned long v, int base) { if (echo) printBaseU(v, base); }
  void print(unsigned int v, int base)  { print((unsigned long)v, base); }
  void print(double v, int digits)      { if (echo) printf("%.*f", digits < 0 ? 0 : digits, v); }
  void println(long v, int base)          { print(v, base); println(); }
  void println(int v, int base)           { print(v, base); println(); }
  void println(unsigned long v, int base) { print(v, base); println(); }
  void println(unsigned int v, int base)  { print(v, base); println(); }
  void println(double v, int digits)      { print(v, digits); println(); }
private:
  static void printBaseU(unsigned long v, int base) {
    if (base < 2 || base > 16) base = 10;
    char b[40]; int n = 0;
    do { b[n++] = "0123456789ABCDEF"[v % (unsigned)base]; v /= (unsigned)base; } while (v);
    while (n) putchar(b[--n]);
  }
  static void printBase(long v, int base) {
    /* Arduino prints negative numbers with a sign only in base 10; other
     * bases show the two's-complement bit pattern. */
    if (base == 10 && v < 0) { putchar('-'); printBaseU(0UL - (unsigned long)v, 10); }
    else printBaseU((unsigned long)v, base);
  }
};
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
extern SerialClass Serial;
