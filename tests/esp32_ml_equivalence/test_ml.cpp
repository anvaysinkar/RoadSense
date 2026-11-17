// test_ml.cpp -- host test: C feature extraction + C forest vs the Python reference.
// Build: see tests/Makefile (target test_ml)   Run: ./test_ml ../../edge_ml/results/test_vectors.txt
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <chrono>
#include "rs_ml_features.h"
#include "rs_ml_model.h"

static std::vector<double> nums(const std::string& line) {
  std::istringstream ss(line); std::string tag; ss >> tag;
  std::vector<double> v; double x; while (ss >> x) v.push_back(x); return v;
}

int main(int argc, char** argv) {
  std::ifstream in(argc > 1 ? argv[1] : "../../edge_ml/results/test_vectors.txt");
  std::string l;
  int n = 0, agree = 0, featBad = 0; double maxProbDiff = 0, maxRel[RSML_NFEAT] = {0};
  double usTotal = 0;
  static RsmlWin w;
  while (std::getline(in, l)) {
    if (l.rfind("EV ", 0) != 0) continue;
    std::istringstream h(l); std::string tag, veh; long t; int nw; long t0; h >> tag >> veh >> t >> nw >> t0;
    std::string lc, lr, lms, lj, lh, lav, lah, lpf, lpp;
    std::getline(in, lc); std::getline(in, lr); std::getline(in, lms); std::getline(in, lj);
    std::getline(in, lh); std::getline(in, lav); std::getline(in, lah); std::getline(in, lpf); std::getline(in, lpp);
    auto c = nums(lc), r = nums(lr), ms = nums(lms), j = nums(lj), hh = nums(lh), av = nums(lav), ah = nums(lah);
    auto pf = nums(lpf), pp = nums(lpp);
    RsmlCal cal{(float)c[0], (float)c[1], (float)c[2], (float)c[3], (float)c[4], (float)c[5], (float)c[6]};
    rsml_reset(w, (uint32_t)t0);
    for (int i = 0; i < nw; i++) rsml_push(w, (uint32_t)ms[i], (float)j[i], (float)hh[i], (float)av[i], (float)ah[i]);
    float f[RSML_NFEAT], p[RSML_NCLASS];
    auto a = std::chrono::high_resolution_clock::now();
    rsml_features(w, cal, (float)r[0], (int)r[1], (float)r[2], (int)r[3], f);
    rsml_predict(f, p);
    auto b = std::chrono::high_resolution_clock::now();
    usTotal += std::chrono::duration<double, std::micro>(b - a).count();
    bool bad = false;
    for (int k = 0; k < RSML_NFEAT; k++) {
      double d = fabs(f[k] - pf[k]) / (fabs(pf[k]) + 1e-3);
      if (d > maxRel[k]) maxRel[k] = d;
      if (d > 1e-3) bad = true;
    }
    featBad += bad;
    int ac = 0, ap = 0;
    for (int k = 0; k < RSML_NCLASS; k++) {
      if (p[k] > p[ac]) ac = k;
      if (pp[k] > pp[ap]) ap = k;
      double d = fabs(p[k] - pp[k]); if (d > maxProbDiff) maxProbDiff = d;
    }
    agree += (ac == ap); n++;
  }
  printf("windows tested: %d\n", n);
  printf("feature vectors off by >0.1%% (relative) in any feature: %d\n", featBad);
  printf("max relative feature error per feature:\n");
  for (int k = 0; k < RSML_NFEAT; k++) printf("  %-12s %.2e\n", RSML_FEATURE_NAMES[k], maxRel[k]);
  printf("max |P_C - P_python| over all classes: %.4f\n", maxProbDiff);
  printf("predicted class identical to Python: %d / %d\n", agree, n);
  printf("host time per event (features + forest): %.1f us\n", usTotal / n);
  return 0;
}
