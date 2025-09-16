// float version — simple
const int NBINS = 5;
float d_block = 22.0;   // cm
float d_free  = 28.0;   // cm
float T_block = 1.0 / (d_block * d_block);
float T_free  = 1.0 / (d_free  * d_free);

enum BinState { BIN_FREE = 0, BIN_BLOCKED = 1 };
BinState binState[NBINS] = {BIN_FREE, BIN_FREE, BIN_FREE, BIN_FREE, BIN_FREE};

void updateBinsFloat(float dist[]) {
  for (int i=0; i<NBINS; ++i) {
    float d = dist[i];              // distance in cm (cap invalid)
    if (d <= 0.0) continue;         // ignore invalid reading
    float H = 1.0f / (d * d);
    if (H > T_block) {
      binState[i] = BIN_BLOCKED;
    } else if (H < T_free) {
      binState[i] = BIN_FREE;
    } // else keep previous state
  }
}
