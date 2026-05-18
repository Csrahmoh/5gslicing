#pragma once

struct QosWeights {
    double K0; // ATR
    double K1; // delay
    double K2; // loss
};

inline QosWeights GetWeights(const std::string& slice)
{
    if (slice == "uRLLC") {
        return {0, 1.0, 10.0};
    }
    else if (slice == "eMBB") {
        return {1e9, 1.0, 10.0}
    }
    else { // mMTC
        return {1.0, 0.5, 10.0};
    }
}

inline double QoSCost(double atrMbps,
                      double delayMs,
                      double cumulativeLoss,
                      const QosWeights& w)
{
    if (atrMbps <= 0.0)
        atrMbps = 0.000001;

    return (w.K0 * (1.0 / atrMbps))
         + (w.K1 * delayMs)
         + (w.K2 * cumulativeLoss);
}