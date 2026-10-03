/* ivp_string.h -- string helpers and string hash
 *
 * Port of ivu_string.hxx / ivu_string.cxx (P_String, p_* helpers) and
 * IVP_U_String_Hash (ivu_string_hash.hxx, ivu_hash.cxx).
 */
#ifndef IVP_STRING_H
#define IVP_STRING_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IVP_WHITESPACE token delimiters */
#define IVP_WHITESPACE " \t,;\n"

/* P_String::uppercase: ASCII a-z → A-Z in place */
void ivp_string_uppercase(char *str);

/* P_String::find_string: first occurrence of key in str, or NULL.
 * upper_case: 0 exact, 1 case-insensitive, 2 '?' in key matches any
 * character, 3 (any other value) both. */
const char *ivp_string_find(const char *str, const char *key, int upper_case);

/* P_String::string_cmp: wildcard compare ('*' any sequence, '?' any char).
 * Returns 0 if equal (reference return conventions, including the raw
 * character value when one string ends first). */
int ivp_string_cmp(const char *str, const char *search, bool upper_case);

/* p_strcmp: NULL-safe strcmp (NULL > any string) */
int ivp_p_strcmp(const char *s1, const char *s2);
/* p_strlen: 0 for NULL */
ptrdiff_t ivp_p_strlen(const char *s);
/* p_atoi / p_atof: 0 for NULL */
int ivp_p_atoi(const char *s);
double ivp_p_atof(const char *s);
/* p_str_tok: strtok that strips '\r' from the token */
char *ivp_p_str_tok(char *a, const char *deli);
/* p_read_first_token: first token of the next non-comment line (static buffer) */
char *ivp_p_read_first_token(FILE *fp);

/* p_strdup / p_make_string with an explicit allocator (NULL = default).
 * Free with ivp_string_free. */
char *ivp_string_dup(const ivp_allocator_t *alloc, const char *s);
char *ivp_string_make(const ivp_allocator_t *alloc, const char *templat, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;
void ivp_string_free(const ivp_allocator_t *alloc, char *s);

/* ── IVP_U_String_Hash ──────────────────────────────────────────────────── *
 * Chained hash: bucket = CRC(key) % size, new keys are prepended (no
 * duplicate check); find returns the first match or not_found_value. */

typedef struct ivp_string_hash_elem {
    struct ivp_string_hash_elem *next;
    void                        *value;
    size_t                       key_len;
    char                         key[];
} ivp_string_hash_elem_t;

typedef struct ivp_string_hash {
    int                       size;
    void                     *not_found_value;
    ivp_string_hash_elem_t  **elems;
    const ivp_allocator_t    *alloc;
} ivp_string_hash_t;

ivp_result_t ivp_string_hash_init(ivp_string_hash_t *h, const ivp_allocator_t *alloc,
                                  int size, void *not_found_value);
void  ivp_string_hash_destroy(ivp_string_hash_t *h);
int   ivp_string_hash_index(const ivp_string_hash_t *h, const char *key);
ivp_result_t ivp_string_hash_add(ivp_string_hash_t *h, const char *key, void *value);
void  ivp_string_hash_remove(ivp_string_hash_t *h, const char *key);
void *ivp_string_hash_find(const ivp_string_hash_t *h, const char *key);

#ifdef __cplusplus
}
#endif

#endif /* IVP_STRING_H */
