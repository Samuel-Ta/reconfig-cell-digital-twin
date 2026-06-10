/*
 * Compatibility shim providing fastcdr Cdr::serialize() overloads for unsigned
 * types that are missing from libfastcdr.so.2 (version 2.2.5).
 *
 * The typesupport libs (rosidl_typesupport_fastrtps_cpp) were compiled against
 * an older fastcdr API that exported individual serialize() overloads for each
 * unsigned primitive type.  In 2.2.5 those became header-only inlines that
 * reference the signed counterparts, so the symbols disappeared from the .so.
 *
 * Each stub below:
 *   - has exactly the mangled name the linker expects
 *   - takes (this*, arg) matching the x86-64 C++ member-function ABI
 *   - delegates to the corresponding signed overload in libfastcdr.so.2
 *   - leaves RAX unchanged so callers get the correct Cdr& return value
 *
 * Build:
 *   gcc -shared -fPIC -o fastcdr_compat.so fastcdr_compat.c -ldl
 * Use:
 *   LD_PRELOAD=/path/to/fastcdr_compat.so
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stddef.h>

/* ── helper: look up an existing serialize overload in libfastcdr ─────────── */

static void* _lookup(const char* sym)
{
    /* Try RTLD_NEXT first (works if libfastcdr is already loaded) */
    void* fn = dlsym(RTLD_NEXT, sym);
    if (fn) return fn;

    /* Fallback: open the library explicitly */
    void* lib = dlopen("libfastcdr.so.2", RTLD_NOW | RTLD_NOLOAD);
    if (!lib) lib = dlopen("libfastcdr.so.2", RTLD_NOW | RTLD_GLOBAL);
    if (lib) fn = dlsym(lib, sym);
    return fn;
}

/* Shorthand typedefs for the real by-value overloads in libfastcdr.so.2 */
typedef void (*fn_c)(void*, char);          /* serialize(char)  */
typedef void (*fn_s)(void*, short);         /* serialize(short) */
typedef void (*fn_i)(void*, int);           /* serialize(int)   */
typedef void (*fn_l)(void*, long);          /* serialize(long)  */

/* Pointers to the real implementations (set once) */
#define DECL_FN(name, type, sym) \
    static type name##_fn = NULL; \
    static type name(void) { \
        if (!name##_fn) name##_fn = (type)_lookup(sym); \
        return name##_fn; \
    }

typedef void (*fn_pkc)(void*, const char*); /* serialize(const char*) */

DECL_FN(get_c,   fn_c,   "_ZN8eprosima7fastcdr3Cdr9serializeEc")
DECL_FN(get_s,   fn_s,   "_ZN8eprosima7fastcdr3Cdr9serializeEs")
DECL_FN(get_i,   fn_i,   "_ZN8eprosima7fastcdr3Cdr9serializeEi")
DECL_FN(get_l,   fn_l,   "_ZN8eprosima7fastcdr3Cdr9serializeEl")
DECL_FN(get_pkc, fn_pkc, "_ZN8eprosima7fastcdr3Cdr9serializeEPKc")

/* ── missing unsigned by-value overloads ────────────────────────────────────
 *
 * Mangling key (x86-64 System V):
 *   h = unsigned char   (uint8_t)
 *   t = unsigned short  (uint16_t)
 *   j = unsigned int    (uint32_t)
 *   m = unsigned long   (uint64_t on LP64)
 *   y = unsigned long long
 */

/* serialize(signed char) → serialize(char) [same representation] */
void _ZN8eprosima7fastcdr3Cdr9serializeEa(void* self, signed char v) {
    fn_c fn = get_c(); if (fn) fn(self, (char)v);
}

/* serialize(char*) → serialize(const char*) [drop const is safe going the other way] */
void _ZN8eprosima7fastcdr3Cdr9serializeEPc(void* self, char* v) {
    fn_pkc fn = get_pkc(); if (fn) fn(self, (const char*)v);
}

/* serialize(uint8_t)  →  serialize(char) */
void _ZN8eprosima7fastcdr3Cdr9serializeEh(void* self, unsigned char v) {
    fn_c fn = get_c(); if (fn) fn(self, (char)v);
}

/* serialize(uint16_t) → serialize(short) */
void _ZN8eprosima7fastcdr3Cdr9serializeEt(void* self, unsigned short v) {
    fn_s fn = get_s(); if (fn) fn(self, (short)v);
}

/* serialize(uint32_t) → serialize(int) */
void _ZN8eprosima7fastcdr3Cdr9serializeEj(void* self, unsigned int v) {
    fn_i fn = get_i(); if (fn) fn(self, (int)v);
}

/* serialize(uint64_t = unsigned long on LP64) → serialize(long) */
void _ZN8eprosima7fastcdr3Cdr9serializeEm(void* self, unsigned long v) {
    fn_l fn = get_l(); if (fn) fn(self, (long)v);
}

/* serialize(unsigned long long) → serialize(long) (same width on LP64) */
void _ZN8eprosima7fastcdr3Cdr9serializeEy(void* self, unsigned long long v) {
    fn_l fn = get_l(); if (fn) fn(self, (long)v);
}

/* ── missing unsigned const-reference overloads ─────────────────────────────
 *
 * On x86-64, a const T& argument to an external function is passed as a
 * pointer (T*) in a general-purpose register.  We dereference it and
 * delegate to the corresponding by-value overload.
 *
 * Mangling: RK<type> = const <type>&
 */

/* serialize(const uint8_t&)  = RKh */
void _ZN8eprosima7fastcdr3Cdr9serializeERKh(void* self, const unsigned char* p) {
    fn_c fn = get_c(); if (fn) fn(self, (char)*p);
}

/* serialize(const uint16_t&) = RKt */
void _ZN8eprosima7fastcdr3Cdr9serializeERKt(void* self, const unsigned short* p) {
    fn_s fn = get_s(); if (fn) fn(self, (short)*p);
}

/* serialize(const uint32_t&) = RKj */
void _ZN8eprosima7fastcdr3Cdr9serializeERKj(void* self, const unsigned int* p) {
    fn_i fn = get_i(); if (fn) fn(self, (int)*p);
}

/* serialize(const uint64_t&) = RKm (unsigned long on LP64) */
void _ZN8eprosima7fastcdr3Cdr9serializeERKm(void* self, const unsigned long* p) {
    fn_l fn = get_l(); if (fn) fn(self, (long)*p);
}

/* serialize(const unsigned long long&) = RKy */
void _ZN8eprosima7fastcdr3Cdr9serializeERKy(void* self, const unsigned long long* p) {
    fn_l fn = get_l(); if (fn) fn(self, (long)*p);
}
