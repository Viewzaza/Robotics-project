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

// On AVR F() puts the literal in flash; on the host it is a no-op.
#define F(x) (x)

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
  void print(int v)             { if (echo) printf("%d", v); }
  void print(long v)            { if (echo) printf("%ld", v); }
  void print(double v)          { if (echo) printf("%g", v); }
  void println()                { if (echo) printf("\n"); }
  void println(const String& v) { if (echo) printf("%s\n", v.c_str()); }
  void println(const char* v)   { if (echo) printf("%s\n", v); }
  void println(int v)           { if (echo) printf("%d\n", v); }
  void println(long v)          { if (echo) printf("%ld\n", v); }
  void println(double v)        { if (echo) printf("%g\n", v); }
};
extern SerialClass Serial;
