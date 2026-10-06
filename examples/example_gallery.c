/* example_gallery.c -- Launcher for the examples
 *
 * Lists every example as a card. Click a card (or select it with the arrow
 * keys and press Enter) to open the example in its own window; H or a right
 * click runs its headless self-check and marks the card with the result, A
 * runs the checks of all examples. The examples are started as separate
 * processes from the directory the gallery lives in, so they keep running
 * when the gallery is closed.
 *
 *   `example_gallery --headless` checks that the card list and the example
 *   executables next to the gallery match (every example has a description).
 *   `example_gallery --screenshot out.png [frames]` saves a window frame.
 *
 * Demonstrates: nothing about the engine; SDL3 processes and the render
 *               module's HUD widgets.
 */
#include <ivp/render/ivp_render.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_process.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct card {
    const char *name;          /* example_<name> */
    int         category;
    const char *text;
} card_t;

static const char *categories[] = { "Physics and controllers", "Geometry", "Larger scenes" };
#define N_CATEGORIES 3

static const card_t cards[] = {
    /* physics and controllers */
    { "freefall",         0, "Balls of different mass and elasticity bounce on the ground." },
    { "cubes",            0, "Stacking, a ramp, friction and elasticity." },
    { "domino",           0, "A chain of toppling tiles." },
    { "bridge",           0, "A plank bridge on ball-socket joints; a heavy box drops on it." },
    { "rope",             0, "A chain pendulum on ball-socket joints." },
    { "hinge",            0, "A cube on a hinge constraint." },
    { "maxforce",         0, "Constraints with a maximum impulse: they give way and recover." },
    { "springs",          0, "A spring bridge and a bungee cord." },
    { "stiff_spring",     0, "Explicit against implicit springs." },
    { "snowflake",        0, "A spring lattice platform with balls on it." },
    { "motor",            0, "A rotation motor spins a blade that sweeps up crates." },
    { "motion_controller", 0, "Drive a body directly: velocity, force or impulse." },
    { "push",             0, "Speed and impulse changes on a cube." },
    { "onthefly",         0, "Change mass, friction and springs at run time." },
    { "force_actuator",   0, "A magnet between two cubes that switches on and off." },
    { "forcefield",       0, "A force field with three modes: gravity, wind, drag." },
    { "black_hole",       0, "An own radial controller pulls 120 bodies together." },
    { "check_distance",   0, "Distance triggers that start a motor." },
    { "phantom",          0, "Trigger volumes that report the objects inside." },
    { "callback",         0, "Object and collision listeners; frozen and revived cores." },
    { "active_value",     0, "Active value chains: a speed that follows another." },
    { "collision_filter", 0, "Collision groups, layers and pair filters." },
    { "raycasting",       0, "The ray solver: nearest hit and all hits." },
    { "universe",         0, "Streaming static objects in and out of the simulation." },
    { "buoyancy",         0, "Bodies of different density in water with a current." },
    { "friction",         0, "Contact friction and the three world friction modes." },
    { "psi_listener",     0, "A PSI listener records trails and an energy chart." },
    /* geometry */
    { "shapes",           1, "Convex hulls: cylinders, cones, prisms, pyramids." },
    { "cut",              1, "Chopping a compact surface into two pieces." },
    { "ring",             1, "A concave ring built from a ledge soup." },
    { "3ds_import",       1, "Loads a .3ds mesh as a physics body." },
    { "terrain",          1, "A heightfield as a compact grid; bodies roll into the valleys." },
    { "concave",          1, "Triangle meshes cut into convex parts; balls stay in the bowl." },
    /* larger scenes */
    { "vehicle",          2, "The four-wheel raycast car on a ground with ramps." },
    { "offroad",          2, "The raycast car over 120 m of hills." },
    { "boat",             2, "A motor boat in a pool, pushing floating crates." },
    { "airboat",          2, "Valve's airboat from a lake up a beach." },
    { "platform",         2, "An elevator, a slider and a turntable carrying crates." },
    { "compound",         2, "Rigid bodies of several objects; break them apart." },
};
#define N_CARDS ((int)(sizeof(cards) / sizeof(cards[0])))

enum { RES_NONE = 0, RES_PASS, RES_FAIL, RES_RUNNING, RES_QUEUED };

#define MAX_JOBS     12
#define MAX_PARALLEL 4
#define CHECK_TIMEOUT_S 60.0

typedef struct job {
    SDL_Process *proc;
    int          card;
    bool         headless;
    double       started;
} job_t;

typedef struct gallery {
    char     dir[1024];
    job_t    jobs[MAX_JOBS];
    int      result[N_CARDS];      /* of the last headless check */
    int      windows[N_CARDS];     /* example windows opened and still running */
    int      selected;
    float    scroll, scroll_max;
    bool     quit;
    bool     prev_lmb, prev_rmb;
    bool     key_prev[SDL_SCANCODE_COUNT];
    float    wheel;
    char     message[160];
} gallery_t;

static void exe_path(const gallery_t *g, const char *name, char *out, size_t size)
{
#ifdef _WIN32
    snprintf(out, size, "%sexample_%s.exe", g->dir, name);
#else
    snprintf(out, size, "%sexample_%s", g->dir, name);
#endif
}

static int free_job(const gallery_t *g)
{
    for (int i = 0; i < MAX_JOBS; i++)
        if (!g->jobs[i].proc) return i;
    return -1;
}

static int running_checks(const gallery_t *g)
{
    int n = 0;
    for (int i = 0; i < MAX_JOBS; i++)
        if (g->jobs[i].proc && g->jobs[i].headless) n++;
    return n;
}

/* Start example `card`: in a window, or its headless check (output discarded). */
static bool launch(gallery_t *g, int card, bool headless, double now)
{
    int slot = free_job(g);
    if (slot < 0) {
        snprintf(g->message, sizeof(g->message), "too many examples running");
        return false;
    }
    char path[1100];
    exe_path(g, cards[card].name, path, sizeof(path));
    const char *args[3] = { path, headless ? "--headless" : NULL, NULL };

    SDL_PropertiesID props = SDL_CreateProperties();
    SDL_SetPointerProperty(props, SDL_PROP_PROCESS_CREATE_ARGS_POINTER, (void *)args);
    if (headless) {
        SDL_SetNumberProperty(props, SDL_PROP_PROCESS_CREATE_STDOUT_NUMBER, SDL_PROCESS_STDIO_NULL);
        SDL_SetNumberProperty(props, SDL_PROP_PROCESS_CREATE_STDERR_NUMBER, SDL_PROCESS_STDIO_NULL);
    }
    SDL_Process *p = SDL_CreateProcessWithProperties(props);
    SDL_DestroyProperties(props);
    if (!p) {
        snprintf(g->message, sizeof(g->message), "cannot start %s: %s", path, SDL_GetError());
        if (headless) g->result[card] = RES_FAIL;
        return false;
    }
    g->jobs[slot].proc = p;
    g->jobs[slot].card = card;
    g->jobs[slot].headless = headless;
    g->jobs[slot].started = now;
    if (headless) g->result[card] = RES_RUNNING;
    else g->windows[card]++;
    g->message[0] = '\0';
    return true;
}

/* Collect finished processes and start queued checks. */
static void poll_jobs(gallery_t *g, double now)
{
    for (int i = 0; i < MAX_JOBS; i++) {
        job_t *j = &g->jobs[i];
        if (!j->proc) continue;
        int code = 0;
        if (j->headless && now - j->started > CHECK_TIMEOUT_S) {
            SDL_KillProcess(j->proc, true);
            SDL_WaitProcess(j->proc, true, &code);
            g->result[j->card] = RES_FAIL;
        } else if (SDL_WaitProcess(j->proc, false, &code)) {
            if (j->headless) g->result[j->card] = code == 0 ? RES_PASS : RES_FAIL;
            else if (g->windows[j->card] > 0) g->windows[j->card]--;
        } else {
            continue;
        }
        SDL_DestroyProcess(j->proc);
        j->proc = NULL;
    }
    for (int c = 0; c < N_CARDS && running_checks(g) < MAX_PARALLEL; c++)
        if (g->result[c] == RES_QUEUED && free_job(g) >= 0) launch(g, c, true, now);
}

/* ── layout ──────────────────────────────────────────────────────────────── */

#define MARGIN   18.0f
#define GAP      10.0f
#define CARD_H   54.0f
#define HEAD_H   26.0f
#define TOP_Y    64.0f
#define BOTTOM_H 54.0f

typedef struct layout {
    int   cols;
    float card_w;
    float y[N_CARDS];              /* top of every card, before scrolling */
    float x[N_CARDS];
    float head_y[N_CATEGORIES];
    float content_h;
} layout_t;

static void make_layout(layout_t *l, int win_w)
{
    l->cols = win_w >= 1500 ? 4 : (win_w >= 900 ? 3 : (win_w >= 600 ? 2 : 1));
    l->card_w = ((float)win_w - 2.0f * MARGIN - GAP * (float)(l->cols - 1)) / (float)l->cols;
    float y = 0.0f;
    for (int cat = 0; cat < N_CATEGORIES; cat++) {
        l->head_y[cat] = y;
        y += HEAD_H;
        int k = 0;
        for (int c = 0; c < N_CARDS; c++) {
            if (cards[c].category != cat) continue;
            l->x[c] = MARGIN + (float)(k % l->cols) * (l->card_w + GAP);
            l->y[c] = y + (float)(k / l->cols) * (CARD_H + GAP);
            k++;
        }
        y += (float)((k + l->cols - 1) / l->cols) * (CARD_H + GAP) + 8.0f;
    }
    l->content_h = y;
}

/* ── events ──────────────────────────────────────────────────────────────── */

static bool on_event(const SDL_Event *ev, void *ud)
{
    gallery_t *g = (gallery_t *)ud;
    if (ev->type == SDL_EVENT_MOUSE_WHEEL) g->wheel += ev->wheel.y;
    return true;                       /* no camera in the gallery */
}

static bool key_pressed(gallery_t *g, SDL_Scancode sc)
{
    const bool *k = SDL_GetKeyboardState(NULL);
    bool now = k[sc], edge = now && !g->key_prev[sc];
    g->key_prev[sc] = now;
    return edge;
}

static void move_selection(gallery_t *g, const layout_t *l, int dx, int dy)
{
    int cur = g->selected, best = cur;
    if (dx) {
        best = cur + dx;
        if (best < 0 || best >= N_CARDS) best = cur;
    } else if (dy) {
        /* the nearest card in the next row up or down, by x */
        float bx = l->x[cur], by = l->y[cur], score = 1e30f;
        for (int c = 0; c < N_CARDS; c++) {
            float ddy = (l->y[c] - by) * (float)dy;
            if (ddy < CARD_H * 0.5f) continue;
            float s = ddy * 4.0f + fabsf(l->x[c] - bx);
            if (s < score) { score = s; best = c; }
        }
    }
    g->selected = best;
}

static void update(gallery_t *g, const layout_t *l, int win_w, int win_h, double now)
{
    float mx, my;
    SDL_MouseButtonFlags btn = SDL_GetMouseState(&mx, &my);
    bool lmb = (btn & SDL_BUTTON_MASK(SDL_BUTTON_LEFT)) != 0, rmb = (btn & SDL_BUTTON_MASK(SDL_BUTTON_RIGHT)) != 0;

    g->scroll -= g->wheel * 48.0f;
    g->wheel = 0.0f;

    bool moved_by_key = false;
    if (key_pressed(g, SDL_SCANCODE_LEFT)  || key_pressed(g, SDL_SCANCODE_A)) { move_selection(g, l, -1, 0); moved_by_key = true; }
    if (key_pressed(g, SDL_SCANCODE_RIGHT) || key_pressed(g, SDL_SCANCODE_D)) { move_selection(g, l, 1, 0); moved_by_key = true; }
    if (key_pressed(g, SDL_SCANCODE_UP)    || key_pressed(g, SDL_SCANCODE_W)) { move_selection(g, l, 0, -1); moved_by_key = true; }
    if (key_pressed(g, SDL_SCANCODE_DOWN)  || key_pressed(g, SDL_SCANCODE_S)) { move_selection(g, l, 0, 1); moved_by_key = true; }
    if (key_pressed(g, SDL_SCANCODE_RETURN) || key_pressed(g, SDL_SCANCODE_SPACE)) launch(g, g->selected, false, now);
    if (key_pressed(g, SDL_SCANCODE_H)) {
        g->result[g->selected] = RES_QUEUED;
    }
    const bool all_down = key_pressed(g, SDL_SCANCODE_X);
    if (all_down)
        for (int c = 0; c < N_CARDS; c++) g->result[c] = RES_QUEUED;
    if (key_pressed(g, SDL_SCANCODE_ESCAPE)) g->quit = true;

    /* the card under the mouse */
    int hover = -1;
    if (my > TOP_Y && my < (float)win_h - BOTTOM_H)
        for (int c = 0; c < N_CARDS; c++) {
            float y = TOP_Y + l->y[c] - g->scroll;
            if (mx >= l->x[c] && mx < l->x[c] + l->card_w && my >= y && my < y + CARD_H) hover = c;
        }
    if (hover >= 0 && !moved_by_key) g->selected = hover;
    if (hover >= 0 && lmb && !g->prev_lmb) launch(g, hover, false, now);
    if (hover >= 0 && rmb && !g->prev_rmb) g->result[hover] = RES_QUEUED;
    g->prev_lmb = lmb;
    g->prev_rmb = rmb;

    /* keep the selected card in view after a key press */
    if (moved_by_key) {
        float top = l->y[g->selected], view_h = (float)win_h - TOP_Y - BOTTOM_H;
        if (top < g->scroll) g->scroll = top - HEAD_H;
        if (top + CARD_H > g->scroll + view_h) g->scroll = top + CARD_H - view_h + GAP;
    }
    (void)win_w;
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void wrap_text(ivp_renderer_t *r, float x, float y, float w, const char *text, const float col[3])
{
    const float cw = ivp_render_text_width("M", 1);
    const int per_line = (int)(w / cw);
    char line[128];
    int len = (int)strlen(text), pos = 0, row = 0;
    while (pos < len && row < 3) {
        int n = len - pos < per_line ? len - pos : per_line;
        if (pos + n < len) {                        /* break at the last space */
            int cut = n;
            while (cut > 0 && text[pos + cut] != ' ') cut--;
            if (cut > 0) n = cut;
        }
        if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;
        memcpy(line, text + pos, (size_t)n);
        line[n] = '\0';
        ivp_render_draw_text_scaled(r, x, y + (float)row * 11.0f, line, col, 1);
        pos += n;
        while (pos < len && text[pos] == ' ') pos++;
        row++;
    }
}

static void draw(gallery_t *g, ivp_renderer_t *r, const layout_t *l, int win_w, int win_h)
{
    const float c_title[3] = { 0.95f, 0.97f, 1.00f }, c_dim[3] = { 0.62f, 0.70f, 0.80f };
    const float c_text[3] = { 0.84f, 0.88f, 0.94f }, c_card[3] = { 0.07f, 0.10f, 0.16f };
    const float c_sel[3] = { 0.16f, 0.30f, 0.50f }, c_edge[3] = { 0.40f, 0.65f, 1.00f };
    const float c_pass[3] = { 0.40f, 0.90f, 0.50f }, c_fail[3] = { 1.00f, 0.35f, 0.30f };
    const float c_run[3] = { 1.00f, 0.85f, 0.30f }, c_head[3] = { 0.55f, 0.80f, 1.00f };

    g->scroll_max = l->content_h - ((float)win_h - TOP_Y - BOTTOM_H);
    if (g->scroll_max < 0.0f) g->scroll_max = 0.0f;
    if (g->scroll > g->scroll_max) g->scroll = g->scroll_max;
    if (g->scroll < 0.0f) g->scroll = 0.0f;

    int passed = 0, failed = 0, running = 0;
    for (int c = 0; c < N_CARDS; c++) {
        passed += g->result[c] == RES_PASS;
        failed += g->result[c] == RES_FAIL;
        running += g->windows[c];
    }

    /* cards (under the header) */
    const float view_top = TOP_Y, view_bottom = (float)win_h - BOTTOM_H;
    for (int cat = 0; cat < N_CATEGORIES; cat++) {
        float y = view_top + l->head_y[cat] - g->scroll;
        if (y + HEAD_H > view_top && y < view_bottom)
            ivp_render_draw_text(r, MARGIN, y + 4.0f, categories[cat], c_head);
    }
    for (int c = 0; c < N_CARDS; c++) {
        float y = view_top + l->y[c] - g->scroll, x = l->x[c];
        if (y + CARD_H < view_top || y > view_bottom) continue;
        const bool sel = c == g->selected;
        ivp_render_draw_rect(r, x, y, l->card_w, CARD_H, sel ? c_sel : c_card, sel ? 0.92f : 0.70f);
        if (sel) ivp_render_draw_rect(r, x, y, 3.0f, CARD_H, c_edge, 1.0f);
        ivp_render_draw_text(r, x + 12.0f, y + 6.0f, cards[c].name, sel ? c_title : c_text);
        wrap_text(r, x + 12.0f, y + 27.0f, l->card_w - 24.0f, cards[c].text, c_dim);

        const char *badge = NULL;
        const float *bcol = c_dim;
        switch (g->result[c]) {
        case RES_PASS:    badge = "OK";      bcol = c_pass; break;
        case RES_FAIL:    badge = "FAIL";    bcol = c_fail; break;
        case RES_RUNNING: badge = "checking"; bcol = c_run; break;
        case RES_QUEUED:  badge = "queued";  bcol = c_dim; break;
        default: break;
        }
        if (badge)
            ivp_render_draw_text_aligned(r, x + l->card_w - 10.0f, y + 6.0f, badge, bcol, IVP_TEXT_ALIGN_RIGHT);
        if (g->windows[c] > 0)
            ivp_render_draw_text_aligned(r, x + l->card_w - 10.0f, y + CARD_H - 20.0f, "open", c_run,
                                         IVP_TEXT_ALIGN_RIGHT);
    }

    /* header and footer plates over the scrolled cards */
    const float plate[3] = { 0.04f, 0.06f, 0.10f };
    ivp_render_draw_panel(r, 0.0f, 0.0f, (float)win_w, TOP_Y - 6.0f, plate, 0.96f);
    ivp_render_draw_panel(r, 0.0f, (float)win_h - BOTTOM_H, (float)win_w, BOTTOM_H, plate, 0.96f);
    ivp_render_draw_text_scaled(r, MARGIN, 10.0f, "libivp examples", c_title, 3);
    {
        char s[128];
        snprintf(s, sizeof(s), "%d examples   %d open   checks: %d ok, %d failed", N_CARDS, running, passed, failed);
        ivp_render_draw_text(r, MARGIN, 40.0f, s, c_dim);
    }
    ivp_render_draw_text(r, MARGIN, (float)win_h - BOTTOM_H + 8.0f,
                         "Click / Enter: open     H / right click: headless check     X: check all     Esc: quit",
                         c_text);
    ivp_render_draw_text(r, MARGIN, (float)win_h - BOTTOM_H + 28.0f,
                         g->message[0] ? g->message : "Arrows / WASD: select     Wheel: scroll", g->message[0] ? c_fail : c_dim);
}

/* ── headless self-check ─────────────────────────────────────────────────── */

static bool check(bool cond, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("[gallery check] %s: %s\n", cond ? "ok  " : "FAIL", buf);
    return cond;
}

/* The cards and the example executables next to the gallery must be the same list. */
static bool self_check(const gallery_t *g)
{
    bool ok = true;
    int found = 0;
    char **files = SDL_GlobDirectory(g->dir, "example_*", 0, &found);
    ok &= check(files != NULL, "listed %d entries in %s", files ? found : 0, g->dir);
    int n_examples = 0;
    for (int i = 0; files && i < found; i++) {
        char name[128];
        snprintf(name, sizeof(name), "%s", files[i] + strlen("example_"));
        char *dot = strrchr(name, '.');
        if (dot) {
            if (strcmp(dot, ".exe") != 0) continue;     /* a library, a debug file ... */
            *dot = '\0';
        }
        if (strcmp(name, "gallery") == 0) continue;
        n_examples++;
        bool listed = false;
        for (int c = 0; c < N_CARDS; c++) listed |= strcmp(cards[c].name, name) == 0;
        ok &= check(listed, "example_%s has a card", name);
    }
    for (int c = 0; c < N_CARDS; c++) {
        char path[1100];
        exe_path(g, cards[c].name, path, sizeof(path));
        SDL_PathInfo info;
        ok &= check(SDL_GetPathInfo(path, &info) && info.type == SDL_PATHTYPE_FILE, "the card %s has an executable",
                    cards[c].name);
        ok &= check(cards[c].category >= 0 && cards[c].category < N_CATEGORIES && cards[c].text[0],
                    "the card %s has a category and a description", cards[c].name);
    }
    ok &= check(n_examples == N_CARDS, "%d executables, %d cards", n_examples, N_CARDS);
    SDL_free(files);
    return ok;
}

/* `--check-all`: the X key without a window. Runs every example's headless check
 * through the same job code and prints the cards that failed. */
static int check_all(gallery_t *g)
{
    for (int c = 0; c < N_CARDS; c++) g->result[c] = RES_QUEUED;
    for (;;) {
        poll_jobs(g, (double)SDL_GetTicksNS() * 1e-9);
        int open = 0;
        for (int c = 0; c < N_CARDS; c++) open += g->result[c] == RES_QUEUED || g->result[c] == RES_RUNNING;
        if (!open) break;
        SDL_Delay(20);
    }
    int failed = 0;
    for (int c = 0; c < N_CARDS; c++)
        if (g->result[c] != RES_PASS) {
            printf("[gallery] %s: check failed\n", cards[c].name);
            failed++;
        }
    printf("[gallery] %d of %d example checks passed\n", N_CARDS - failed, N_CARDS);
    return failed ? 1 : 0;
}

int main(int argc, char **argv)
{
    gallery_t *g = (gallery_t *)calloc(1, sizeof(*g));
    if (!g) return 1;
    const char *base = SDL_GetBasePath();
    snprintf(g->dir, sizeof(g->dir), "%s", base ? base : "./");

    const char *shot = NULL;
    int shot_frames = 30;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--headless") == 0 || strcmp(argv[i], "-H") == 0) {
            bool ok = self_check(g);
            printf("[gallery] self-check %s\n", ok ? "PASSED" : "FAILED");
            free(g);
            return ok ? 0 : 1;
        }
        if (strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            shot = argv[++i];
            if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9') shot_frames = atoi(argv[++i]);
        }
        if (strcmp(argv[i], "--check-all") == 0) {
            int rc = check_all(g);
            free(g);
            return rc;
        }
    }
    if (getenv("IVP_EXAMPLE_HEADLESS")) {
        bool ok = self_check(g);
        printf("[gallery] self-check %s\n", ok ? "PASSED" : "FAILED");
        free(g);
        return ok ? 0 : 1;
    }

    ivp_render_config_t rcfg;
    ivp_render_config_init(&rcfg);
    rcfg.width = 1280;
    rcfg.height = 720;
    rcfg.title = "libivp examples";
    rcfg.vsync = true;
    ivp_renderer_t *r = NULL;
    ivp_renderer_create(&r, &rcfg);
    if (!r) {
        fprintf(stderr, "renderer init failed\n");
        free(g);
        return 1;
    }
    const float top[3] = { 0.06f, 0.10f, 0.18f }, bottom[3] = { 0.12f, 0.18f, 0.28f };
    ivp_render_set_backdrop(r, top, bottom);
    ivp_renderer_set_clear_color(r, 0.08f, 0.12f, 0.20f, 1.0f);
    ivp_renderer_set_event_callback(r, on_event, g);

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    int frame = 0;
    while (!g->quit && ivp_renderer_begin_frame(r, &cam)) {
        int w, h;
        ivp_renderer_get_size(r, &w, &h);
        layout_t l;
        make_layout(&l, w);
        const double now = ivp_renderer_get_time(r);
        poll_jobs(g, now);
        update(g, &l, w, h, now);
        draw(g, r, &l, w, h);
        if (shot && frame == shot_frames - 1) ivp_render_request_capture(r, shot);
        ivp_renderer_end_frame(r);
        frame++;
        if (shot && frame >= shot_frames) {
            printf("[gallery] screenshot %s\n", shot);
            break;
        }
    }
    for (int i = 0; i < MAX_JOBS; i++) {
        if (!g->jobs[i].proc) continue;
        if (g->jobs[i].headless) SDL_KillProcess(g->jobs[i].proc, true);   /* checks stop with the gallery */
        SDL_DestroyProcess(g->jobs[i].proc);                               /* windows keep running */
    }
    ivp_renderer_destroy(r);
    free(g);
    return 0;
}
