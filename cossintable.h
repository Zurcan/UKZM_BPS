
// declaration for Cos and Sin tables
extern const signed short CosSinTable [20480];

const signed short *Cos = &CosSinTable[0];
const signed short *Sin = &CosSinTable[4096];
#define CosSinArgMask 0x3FFF
#define CosSinShift 14
#define CosSinCount (1 << CosSinShift)
