#include "ina226.h"

#define REG_CONFIG   0x00
#define REG_SHUNT    0x01
#define REG_BUS      0x02
#define REG_MASK     0x06

static ina_write_fn s_wr;
static ina_read_fn  s_rd;
static uint8_t      s_addr;
static float        s_r_mohm = 2.0f;   /* 采样电阻，单位 mΩ，按你的板子改 */

static int rd16(uint8_t reg, uint16_t *out)
{
    uint8_t b[2];
    if (s_rd(s_addr, reg, b, 2) != 0) return -1;
    *out = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return 0;
}

static int wr16(uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
    return s_wr(s_addr, reg, b, 2);
}

int ina226_init(ina_write_fn wr, ina_read_fn rd, uint8_t addr, float r_mohm)
{
    s_wr = wr;
    s_rd = rd;
    s_addr = addr;
    if (r_mohm > 0.01f) s_r_mohm = r_mohm;

    /* AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, 连续转换 */
    if (wr16(REG_CONFIG, 0x1507) != 0) return -1;
    /* 清掉 Mask/Enable，避免 ALERT 干扰 */
    wr16(REG_MASK, 0x0000);
    return 0;
}

int ina226_read(uint16_t *mv, int32_t *ma)
{
    uint16_t raw;
    int32_t shunt_uv;

    if (rd16(REG_BUS, &raw) != 0) return -1;
    if (mv) *mv = (uint16_t)((float)raw * 1.25f);

    if (rd16(REG_SHUNT, &raw) != 0) return -1;
    shunt_uv = (int32_t)(int16_t)raw * 5 / 2;      /* 2.5 µV/LSB，整数近似 */
    if (ma) *ma = (int32_t)((float)shunt_uv / s_r_mohm);
    return 0;
}
