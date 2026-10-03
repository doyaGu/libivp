/* ivp_string.c -- port of ivu_string.cxx and IVP_U_String_Hash (ivu_hash.cxx) */
#include <ivp/utility/ivp_string.h>
#include <ivp/utility/ivp_hash.h>
#include <ivp/base/ivp_assert.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

void ivp_string_uppercase(char *str) {
    char c;
    while ((c = *str)) {
        if ((c <= 'z') && (c >= 'a')) *str = (char)(c - 'a' + 'A');
        str++;
    }
}

static inline int upc(char c) { return toupper((unsigned char)c); }

const char *ivp_string_find(const char *str, const char *key, int upper_case) {
    /* checks for a substring in another string
       upper_case = 0 -> exact match
                  = 1 -> a==A
                  = 2 -> a==A && a==? (reference: only '?' wildcard)
                  other -> both */
    if (str == NULL) return NULL;
    const char *p1, *p2;
    char b;
    for (p1 = str, p2 = key; *p1;) {
        if (!(b = *p2)) {
            return str;
        }
        bool match;
        switch (upper_case) {
        case 0:  match = (b == *p1); break;
        case 1:  match = (upc(*p1) == upc(b)); break;
        case 2:  match = (b == *p1 || b == '?'); break;
        default: match = (upc(*p1) == upc(b) || b == '?'); break;
        }
        if (match) {
            p1++;
            p2++;
        } else {
            p2 = key;
            p1 = (++str);
        }
    }
    if (!*p2) return str;
    return NULL;
}

int ivp_string_cmp(const char *str, const char *search, bool upper_case) {
    /* '*' wildcard in search string, '?' any character */
    const char *p1, *p2;
    char a, b, *d;
    int i;
    char fsbuf[256];

    p1 = str;
    if (!p1) return -1;
    p2 = search;
    while (1) {
        a = *p1;
        b = *p2;
        if (b == '*') {
            if (!p2[1]) return 0; /* last character is wildcard */
            i = 0;
            d = fsbuf;
            for (p2++; (b = *p2) && (b != '*');) {
                *(d++) = b;
                p2++;
                i++;
                if (i > 250) break;
            }
            if (*p2 != '*') {
                /* p1 += strlen(p1) - i; if (p1 < str) return -1; */
                ptrdiff_t off = (p1 - str) + (ptrdiff_t)strlen(p1) - (ptrdiff_t)i;
                if (off < 0) return -1; /* neither less or greater */
                p1 = str + off; /* check the end of the string */
                p2 -= i;
            } else {
                *d = 0;
                p1 = ivp_string_find(p1, fsbuf, (upper_case ? 1 : 0) + 2);
                if (!p1) return -1;
                p1 += i;
            }
            continue;
        } else {
            if (!a) return b;
            if (!b) return a;
            if (b != '?') {
                if (upper_case) {
                    if (upc(a) != upc(b)) return 1;
                } else {
                    if (a != b) return 1;
                }
            }
        }
        p1++;
        p2++;
    }
}

int ivp_p_strcmp(const char *s1, const char *s2) {
    if (s1 == NULL) {
        if (s2 == NULL) return 0;
        return 1;
    }
    if (s2 == NULL) return -1;
    return strcmp(s1, s2);
}

ptrdiff_t ivp_p_strlen(const char *s) {
    if (!s || s[0] == 0) return 0;
    return (ptrdiff_t)strlen(s);
}

int ivp_p_atoi(const char *s) {
    if (!s) return 0;
    return atoi(s);
}

double ivp_p_atof(const char *s) {
    if (!s) return 0.0;
    return atof(s);
}

char *ivp_p_str_tok(char *a, const char *deli) {
    char *temp = strtok(a, deli);
    if (!temp) return NULL;
    for (int i = 0; temp[i]; i++) {
        if (temp[i] == 13) temp[i] = 0;
    }
    return temp;
}

char *ivp_p_read_first_token(FILE *fp) {
    static char buffer[1024];
    while (fgets(buffer, 1000, fp)) {
        if (buffer[0] == '#') continue; /* comment */
        char *tok = ivp_p_str_tok(buffer, IVP_WHITESPACE);
        if (!tok) continue;
        return tok;
    }
    return NULL;
}

char *ivp_string_dup(const ivp_allocator_t *alloc, const char *s) {
    if (!s) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    size_t len = strlen(s) + 1;
    char *s2 = (char *)ivp_alloc(alloc, len);
    if (s2) memcpy(s2, s, len);
    return s2;
}

#define MAX_MAKE_STRING_LEN 10000

char *ivp_string_make(const ivp_allocator_t *alloc, const char *templat, ...) {
    if (!templat) return NULL;
    char buffer[MAX_MAKE_STRING_LEN];
    va_list parg;
    va_start(parg, templat);
    vsnprintf(buffer, sizeof(buffer), templat, parg);
    va_end(parg);
    return ivp_string_dup(alloc, buffer);
}

void ivp_string_free(const ivp_allocator_t *alloc, char *s) {
    if (!s) return;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_free(alloc, s, strlen(s) + 1);
}

/* ── IVP_U_String_Hash ──────────────────────────────────────────────────── */

ivp_result_t ivp_string_hash_init(ivp_string_hash_t *h, const ivp_allocator_t *alloc,
                                  int size, void *not_found_value) {
    memset(h, 0, sizeof(*h));
    if (size <= 0) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();
    h->alloc = alloc;
    h->size = size;
    h->not_found_value = not_found_value;
    h->elems = (ivp_string_hash_elem_t **)ivp_alloc(alloc, sizeof(void *) * (size_t)size);
    if (!h->elems) return IVP_ERR_OUT_OF_MEMORY;
    memset(h->elems, 0, sizeof(void *) * (size_t)size);
    return IVP_OK;
}

static void free_elem(const ivp_string_hash_t *h, ivp_string_hash_elem_t *el) {
    ivp_free(h->alloc, el, sizeof(*el) + el->key_len + 1);
}

void ivp_string_hash_destroy(ivp_string_hash_t *h) {
    if (!h->elems) return;
    for (int i = 0; i < h->size; i++) {
        ivp_string_hash_elem_t *next;
        for (ivp_string_hash_elem_t *el = h->elems[i]; el; el = next) {
            next = el->next;
            free_elem(h, el);
        }
    }
    ivp_free(h->alloc, h->elems, sizeof(void *) * (size_t)h->size);
    h->elems = NULL;
}

int ivp_string_hash_index(const ivp_string_hash_t *h, const char *key) {
    unsigned int index = ivp_hash_crc(key, strlen(key));
    return (int)(index % (unsigned int)h->size);
}

ivp_result_t ivp_string_hash_add(ivp_string_hash_t *h, const char *key, void *value) {
    int i = ivp_string_hash_index(h, key);
    size_t keysize = strlen(key);
    ivp_string_hash_elem_t *el =
        (ivp_string_hash_elem_t *)ivp_alloc(h->alloc, sizeof(*el) + keysize + 1);
    if (!el) return IVP_ERR_OUT_OF_MEMORY;
    memcpy(el->key, key, keysize + 1);
    el->key_len = keysize;
    el->next = h->elems[i];
    h->elems[i] = el;
    el->value = value;
    return IVP_OK;
}

void ivp_string_hash_remove(ivp_string_hash_t *h, const char *key) {
    int i = ivp_string_hash_index(h, key);
    ivp_string_hash_elem_t *last_el = NULL;
    for (ivp_string_hash_elem_t *el = h->elems[i]; el; el = el->next) {
        if (!strcmp(el->key, key)) {
            if (last_el) {
                last_el->next = el->next;
            } else {
                h->elems[i] = el->next;
            }
            free_elem(h, el);
            return;
        }
        last_el = el;
    }
}

void *ivp_string_hash_find(const ivp_string_hash_t *h, const char *key) {
    int i = ivp_string_hash_index(h, key);
    IVP_ASSERT(i >= 0);
    for (ivp_string_hash_elem_t *el = h->elems[i]; el; el = el->next) {
        if (!strcmp(el->key, key)) return el->value;
    }
    return h->not_found_value;
}
