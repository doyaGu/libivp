/* Shared fixture coordinates; height=0 preserves the historical 8x8 pile. */
#ifndef BENCH_SCENE_H
#define BENCH_SCENE_H
static int bench_contact_width(int count, int height)
{
    if (!height) return 8;
    int columns = (count + height - 1) / height;
    int width = 1;
    while (width * width < columns) ++width;
    return width;
}

static void bench_contact_position(int i, int count, int height, double p[3])
{
    int width = bench_contact_width(count, height);
    int columns = height ? (count + height - 1) / height : 64;
    int column = i % columns;
    p[0] = (column % width - width / 2) * 1.05;
    p[1] = 1.8 - (i / columns) * 1.0;
    p[2] = (column / width - width / 2) * 1.05;
}

static float bench_contact_ground_extent(int count, int height)
{
    if (!height) return 20.0f;
    double extent = bench_contact_width(count, height) * 1.05 + 2.0;
    return (float)(extent < 20.0 ? 20.0 : extent);
}
#endif
