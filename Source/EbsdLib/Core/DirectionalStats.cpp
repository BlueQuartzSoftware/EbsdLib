#include "DirectionalStats.hpp"

#include "EbsdLib/Orientation/Quaternion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

using namespace ebsdlib;

namespace
{ // ---------- Random helpers mirroring the Fortran routines ----------

template <class T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
T BesselI0(T x) noexcept
{
  // Coefficients (cast to T)
  const T p1 = T(1.0);
  const T p2 = T(3.5156229);
  const T p3 = T(3.0899424);
  const T p4 = T(1.2067492);
  const T p5 = T(0.2659732);
  const T p6 = T(0.0360768); // 0.360768D-1
  const T p7 = T(0.0045813); // 0.45813D-2

  const T q1 = T(0.39894228);
  const T q2 = T(0.01328592);  // 0.1328592D-1
  const T q3 = T(0.00225319);  // 0.225319D-2
  const T q4 = T(-0.00157565); // -0.157565D-2
  const T q5 = T(0.00916281);  // 0.916281D-2
  const T q6 = T(-0.02057706); // -0.2057706D-1
  const T q7 = T(0.02635537);  // 0.2635537D-1
  const T q8 = T(-0.01647633); // -0.1647633D-1
  const T q9 = T(0.00392377);  // 0.392377D-2

  const T ax = std::abs(x);
  if(ax < T(3.75))
  {
    const T y = (x / T(3.75)) * (x / T(3.75));
    return p1 + y * (p2 + y * (p3 + y * (p4 + y * (p5 + y * (p6 + y * p7)))));
  }

  const T y = T(3.75) / ax;
  const T bx = std::exp(ax) / std::sqrt(ax);
  const T a = q1 + y * (q2 + y * (q3 + y * (q4 + y * (q5 + y * (q6 + y * (q7 + y * (q8 + y * q9)))))));
  return a * bx;
}

template <class T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
T BesselI1(T x) noexcept
{
  // Coefficients (from the Fortran routine), cast to T
  const T P1 = T(0.5);
  const T P2 = T(0.87890594);
  const T P3 = T(0.51498869);
  const T P4 = T(0.15084934);
  const T P5 = T(0.02658733); // 0.2658733D-1
  const T P6 = T(0.00301532); // 0.301532D-2
  const T P7 = T(0.00032411); // 0.32411D-3

  const T Q1 = T(0.39894228);
  const T Q2 = T(-0.03988024); // -0.3988024D-1
  const T Q3 = T(-0.00362018); // -0.362018D-2
  const T Q4 = T(0.00163801);  // 0.163801D-2
  const T Q5 = T(-0.01031555); // -0.1031555D-1
  const T Q6 = T(0.02282967);  // 0.2282967D-1
  const T Q7 = T(-0.02895312); // -0.2895312D-1
  const T Q8 = T(0.01787654);  // 0.1787654D-1
  const T Q9 = T(-0.00420059); // -0.420059D-2

  const T ax = std::abs(x);
  if(ax < T(3.75))
  {
    const T y = (x / T(3.75)) * (x / T(3.75));
    // Small-argument series: note the leading factor x
    return x * (P1 + y * (P2 + y * (P3 + y * (P4 + y * (P5 + y * (P6 + y * P7))))));
  }

  const T y = T(3.75) / ax;
  const T bx = std::exp(ax) / std::sqrt(ax);
  const T a = Q1 + y * (Q2 + y * (Q3 + y * (Q4 + y * (Q5 + y * (Q6 + y * (Q7 + y * (Q8 + y * Q9)))))));
  // As in the provided Fortran, this branch does not apply sign(x).
  return a * bx;
}

// Assumes templated BesselI0<T> and BesselI1<T> are available.

// Modified Bessel function of the first kind, integer order N: I_N(x)
template <class T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
T BesselIn(T x, int N) noexcept
{
  // Special cases
  if(N == 0)
  {
    return BesselI0(x);
  }
  if(N == 1)
  {
    return BesselI1(x);
  }
  if(x == T(0))
  {
    return T(0);
  }

  // Constants (mirroring Fortran)
  constexpr int iacc = 40;
  const T bigno = T(1e10);
  const T bigni = T(1e-10);

  // Set up Miller downward recurrence
  const T tox = T(2) / x;
  T bip = T(0);
  T bi = T(1);
  T bessi = T(0);

  // M = 2 * ( N + int(sqrt(IACC * N)) )
  const int M = 2 * (N + static_cast<int>(std::sqrt(static_cast<T>(iacc * N))));

  for(int j = M; j >= 1; --j)
  {
    const T bim = bip + (static_cast<T>(j) * tox * bi);
    bip = bi;
    bi = bim;

    if(std::abs(bi) > bigno)
    {
      bi *= bigni;
      bip *= bigni;
      bessi *= bigni;
    }
    if(j == N)
    {
      bessi = bip;
    }
  }

  // Normalize using I0(x)
  bessi = bessi * BesselI0(x) / bi;
  return bessi;
}

// ---- small helper for I2 using a stable recurrence + small-x series ----
template <class T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
T BesselI2(T x) noexcept
{
  const T ax = std::abs(x);
  if(ax < T(1e-6))
  {
    // Series: I2(x) = x^2/8 + x^4/96 + O(x^6)
    const T x2 = x * x;
    return (x2 * (T(1) / T(8))) + (x2 * x2 * (T(1) / T(96)));
  }
  // Recurrence: I_{ν-1} - I_{ν+1} = (2ν/x) I_ν, with ν=1 -> I0 - I2 = (2/x) I1
  // => I2 = I0 - (2/x) * I1
  return BesselI0(x) - ((T(2) / x) * BesselI1(x));
}

// Park-Miller "minimal standard" PRNG — matches the Fortran r8_uniform_01 exactly.
// Reference: Lewis, Goodman, Miller (1969); Schrage's method to avoid overflow.
//   seed = 16807 * seed mod (2^31 - 1)
//   result = seed / (2^31 - 1)
double r8_uniform_01(uint32_t& seed)
{
  int32_t s = static_cast<int32_t>(seed);
  const int32_t k = s / 127773;
  s = (16807 * (s - k * 127773)) - (k * 2836);
  if(s < 0)
  {
    s += 2147483647;
  }
  seed = static_cast<uint32_t>(s);
  return static_cast<double>(s) * 4.656612875e-10;
}

void r8vec_uniform_01(int m, uint32_t& seed, std::vector<double>& r)
{
  r.resize(m);
  for(int i = 0; i < m; ++i)
  {
    r[i] = r8_uniform_01(seed);
  }
}

// Faithful port of the Fortran r8vec_normal_01
void r8vec_normal_01(int n, uint32_t& seed, double* x)
{
  if(n <= 0)
  {
    return;
  }

  constexpr double r8_pi = 3.141592653589793238462643383279502884;

  int x_lo_index = 0; // Fortran 1-based -> 0-based
  int x_hi_index = n - 1;

  // If we need just one new value, handle it directly.
  if(x_hi_index - x_lo_index + 1 == 1)
  {
    double r1 = r8_uniform_01(seed);
    if(r1 <= 0.0)
    {
      r1 = std::numeric_limits<double>::min();
    }
    double r2 = r8_uniform_01(seed);
    x[x_hi_index] = std::sqrt(-2.0 * std::log(r1)) * std::cos(2.0 * r8_pi * r2);
    return;
  }

  // If we require an even number of values:
  if((x_hi_index - x_lo_index) % 2 == 1)
  {
    const int m = (x_hi_index - x_lo_index + 1) / 2;
    std::vector<double> r;
    r8vec_uniform_01(2 * m, seed, r);

    // Fill pairs: cos for even positions, sin for odd positions
    for(int k = 0; k < m; ++k)
    {
      const double u1 = r[2 * k];
      const double u2 = r[2 * k + 1];
      const double rho = std::sqrt(-2.0 * std::log(std::max(u1, std::numeric_limits<double>::min())));
      const double theta = 2.0 * r8_pi * u2;

      x[x_lo_index + 2 * k] = rho * std::cos(theta);
      x[x_lo_index + 2 * k + 1] = rho * std::sin(theta);
    }
    return;
  }

  // Otherwise, odd number of values (>1): generate an even number, fill all but the last,
  // and compute the final one specially.
  {
    x_hi_index = x_hi_index - 1;
    const int m = (x_hi_index - x_lo_index + 1) / 2 + 1;

    std::vector<double> r;
    r8vec_uniform_01(2 * m, seed, r);

    // Use the first (m-1) pairs to fill up to x_hi_index
    for(int k = 0; k < m - 1; ++k)
    {
      const double u1 = r[2 * k];
      const double u2 = r[2 * k + 1];
      const double rho = std::sqrt(-2.0 * std::log(std::max(u1, std::numeric_limits<double>::min())));
      const double theta = 2.0 * r8_pi * u2;

      x[x_lo_index + 2 * k] = rho * std::cos(theta);
      x[x_lo_index + 2 * k + 1] = rho * std::sin(theta);
    }

    // Final value uses the very last pair (2*m-2, 2*m-1) in 0-based indexing
    const double u1 = r[2 * m - 2];
    const double u2 = r[2 * m - 1];
    const double rho = std::sqrt(-2.0 * std::log(std::max(u1, std::numeric_limits<double>::min())));
    const double theta = 2.0 * r8_pi * u2;

    x[n - 1] = rho * std::cos(theta);
  }
}

} // anonymous namespace

DirectionalStats::DirectionalStats(const std::string& DSType, LaueOps::Pointer laueOps)
: m_DSType(DSType)
, m_LaueOps(laueOps)
{
  if(m_DSType.empty())
  {
    return;
  }

  // Build a lookup table for the Ap(u) parameter used in the M-step.
  // For VMF: ratio I2/I1 is tabulated for kappa in [0.001, 35].
  // For Watson: ratio involving I0,I1 at half-kappa is tabulated for kappa in [0.001, 35].
  // Above these ranges an analytical approximation is used instead.
  m_ApNum = 35000;
  m_XAp.resize(m_ApNum);
  m_YAp.resize(m_ApNum);

  for(int k = 0; k < m_ApNum; ++k)
  {
    m_XAp[k] = 0.001 + static_cast<double>(k) * 0.001;
  }

  if(m_DSType == "VMF")
  {
    for(int k = 0; k < m_ApNum; ++k)
    {
      const double x = m_XAp[k];
      const double denom = static_cast<double>(BesselI1(x));
      const double numer = static_cast<double>(BesselI2(x));
      const double safeDen = (std::abs(denom) > 1e-300) ? denom : std::numeric_limits<double>::min();
      m_YAp[k] = numer / safeDen;
    }
  }
  else if(m_DSType == "WAT")
  {
    for(int k = 0; k < m_ApNum; ++k)
    {
      const double x = m_XAp[k];
      const double xh = 0.5 * x;
      const double I1h = static_cast<double>(BesselI1(xh));
      const double I0h = static_cast<double>(BesselI0(xh));
      double denom = (I0h - I1h) * x;
      if(!(std::abs(denom) > 0.0))
      {
        denom = std::numeric_limits<double>::min();
      }
      m_YAp[k] = I1h / denom;
    }
  }
}

DirectionalStats::~DirectionalStats() = default;

// Expectation-maximization estimation of mean direction (muhat) and concentration
// parameter (kappahat) for the VMF or Watson distribution on the quaternion sphere.
// Input quaternions must be set via setQuatArray() before calling.
void DirectionalStats::EMforDS(uint32_t& seed, QuatD& muhat, double& kappahat, bool verbose)
{

  // array sizes
  const int N = this->getN();
  const int pmdims = m_LaueOps->getNumSymOps();
  const int numEm = m_NumEM;
  const int numIter = m_NumIter;

  if(verbose)
  {
    std::printf(" Starting EMforDS routine\n");
    std::printf(" N=%d, Pmdims=%d, NumEM=%d, NumIter=%d\n", N, pmdims, numEm, numIter);
  }

  std::vector<QuatD> muAll(numEm);
  std::vector<double> kappaAll(numEm, 0.0);
  std::vector<double> lAll(numEm, 0.0);

  // Run EM from multiple random starting points to avoid local maxima
  for(int init = 0; init < numEm; ++init)
  {
    // Random unit quaternion as starting guess for Mu
    std::array<double, 4> v;
    r8vec_normal_01(4, seed, v.data());
    QuatD mu = QuatD(v[1], v[2], v[3], v[0]).normalize().getPositiveOrientation();

    double kappa = 30.0;
    std::vector<double> q(numIter, 0.0);
    std::vector<double> l(numIter, 0.0);

    if(verbose)
    {
      std::printf("\n starting iteration %4d\n", init + 1);
      std::printf(" Initial guess for Mu (wxyz): %12.8f %12.8f %12.8f %12.8f\n", mu.w(), mu.x(), mu.y(), mu.z());
      std::printf(" Initial guess for Kappa : %12.8f\n", kappa);
    }

    double Qi = 0.0, Li = 0.0;
    for(int i = 0; i < numIter; ++i)
    {
      // E-step
      std::vector<double> const r = this->Estep_(mu, kappa);

      // M-step — returns MuKa
      std::array<double, 5> muKa = this->Mstep_(r, N, pmdims);

      // Q and Likelihood
      this->getQandL_(muKa, r, Qi, Li);
      q[i] = Qi;
      l[i] = Li;

      if(verbose)
      {
        std::printf(" inner loop : %4d\n", i + 1);
        std::printf("   Li : %16.8f\n", Li);
        std::printf("   Qi : %16.8f\n", Qi);
        std::printf("   Current guess for MuKa (wxyz,kappa): %12.8f %12.8f %12.8f %12.8f %12.8f\n", muKa[3], muKa[0], muKa[1], muKa[2], muKa[4]);
      }

      muAll[init] = QuatD(muKa[0], muKa[1], muKa[2], muKa[3]);
      kappaAll[init] = muKa[4];
      lAll[init] = l[i];

      mu = muAll[init];
      kappa = kappaAll[init];

      // Convergence check
      if(i >= 1 && std::fabs(q[i] - q[i - 1]) < 0.01)
      {
        if(verbose)
        {
          std::printf(" Exiting inner loop : %4d\n", i + 1);
        }
        break;
      }
    }
  }

  // Select the starting point that achieved the highest likelihood
  int dd = 0;
  {
    double best = -std::numeric_limits<double>::infinity();
    for(int i = 0; i < numEm; ++i)
    {
      if(lAll[i] > best)
      {
        best = lAll[i];
        dd = i;
      }
    }
  }

  if(verbose)
  {
    std::printf(" best fit init: %d\n", dd + 1);
  }

  QuatD mu = muAll[dd];
  mu.positiveOrientation();
  kappahat = kappaAll[dd];

  // Map mu into the fundamental zone by testing all symmetry equivalents
  for(int i = 0; i < pmdims; ++i)
  {
    QuatD const qu = (mu * m_LaueOps->getQuatSymOp(i)).getPositiveOrientation();

    if(m_LaueOps->IsInsideFZ(qu, m_LaueOps->getFZType(), m_LaueOps->getAxisOrderingType()))
    {
      muhat = qu;
      return;
    }
  }

  muhat = mu;
  muhat.positiveOrientation();
}

// Computes the E-step responsibilities matrix R (size N x Pmdims), column-major.
// For each symmetry operator j, computes the density of each quaternion under
// the distribution centered at Mu*symOp(j), then normalizes rows to sum to 1.
std::vector<double> DirectionalStats::Estep_(const QuatD& Mu, double Kappa) const
{
  const int N = this->getN();
  const int Pmdims = m_LaueOps->getNumSymOps();

  std::vector<double> R(static_cast<size_t>(N) * Pmdims, 0.0);
  std::vector<double> rowsum(N, 0.0);

  // C = logCp_(Kappa)
  const double C = this->logCp_(Kappa);

  // Fill columns j = 0..Pmdims-1 (Fortran j=1..Pmdims)
  for(int j = 0; j < Pmdims; ++j)
  {
    QuatD PmMu = Mu * m_LaueOps->getQuatSymOp(j);

    // Column vector of densities (length N)
    std::vector<double> col = this->Density_(PmMu, Kappa, C);
    if(static_cast<int>(col.size()) != N)
    {
      // Defensive: if Density_ returns unexpected length, truncate/pad.
      col.resize(N, 0.0);
    }

    // Store column in R (column-major) and accumulate row-sums
    const size_t base = static_cast<size_t>(j) * N;
    for(int i = 0; i < N; ++i)
    {
      const double val = col[i];
      R[base + i] = val;
      rowsum[i] += val;
    }
  }

  // Normalize rows: R(i, j) /= sum_j R(i, j)
  // Guard against zero row-sum (very unlikely but defensive)
  constexpr double tiny = 1e-300;
  for(int j = 0; j < Pmdims; ++j)
  {
    const size_t base = static_cast<size_t>(j) * N;
    for(int i = 0; i < N; ++i)
    {
      const double denom = (rowsum[i] > tiny) ? rowsum[i] : tiny;
      R[base + i] /= denom;
    }
  }

  return R;
}

// Computes the density of each stored quaternion under the distribution
// centered at mu with concentration kappa and log-normalization constant C.
//   VMF: y_j = exp( C + kappa * dot(mu, q_j) )
//   WAT: y_j = exp( C + kappa * dot(mu, q_j)^2 )
std::vector<double> DirectionalStats::Density_(const QuatD& mu, double kappa, double C) const
{
  const int N = this->getN();
  std::vector<double> y(N);

  const bool isWAT = (m_DSType == "WAT");

  for(int j = 0; j < N; ++j)
  {
    const double dp = mu.dotProduct(m_XQuats[j]);

    if(isWAT)
    {
      y[j] = std::exp(C + kappa * (dp * dp));
    }
    else
    {
      y[j] = std::exp(C + kappa * dp);
    }
  }

  return y;
}

// Log normalization constant for the VMF or Watson distribution on S^3.
double DirectionalStats::logCp_(double kappa) const
{
  constexpr double C = -3.675754132818690967;   // ln(1/(2*pi)^2)
  constexpr double C2 = 4.1746562059854348688;  // ln(512/sqrt(2)/pi^(3/2))
  constexpr double C2W = 5.4243952068443172530; // ln(128*sqrt(pi))

  if(m_DSType == "WAT")
  {
    if(kappa > 20.0)
    {
      const double num_log = 4.5 * std::log(kappa);
      const double den_poly = 525.0 + 4.0 * kappa * (45.0 + 8.0 * kappa * (3.0 + 4.0 * kappa));
      return C2W - kappa + (num_log - std::log(den_poly));
    }

    const double x = 0.5 * kappa;
    const double I0 = BesselI0(x);
    const double I1 = BesselI1(x);
    double diff = I0 - I1;
    if(!(diff > 0.0))
    {
      diff = std::numeric_limits<double>::min();
    }
    return -0.5 * kappa - std::log(diff);
  }

  // VMF (default)
  if(kappa > 30.0)
  {
    const double num_log = 4.5 * std::log(kappa);
    const double den_poly = -105.0 + 8.0 * kappa * (-15.0 + 16.0 * kappa * (-3.0 + 8.0 * kappa));
    return C2 - kappa + (num_log - std::log(std::abs(den_poly)));
  }

  const double I1 = BesselI1(kappa);
  const double denom = (I1 > 0.0) ? I1 : std::numeric_limits<double>::min();
  return C + std::log(kappa / denom);
}

std::array<double, 5> DirectionalStats::Mstep_(const std::vector<double>& R, int N, int Pmdims) const
{
  std::array<double, 5> MuKa{0, 0, 0, 0, 0}; // [x,y,z,w,kappa]
  auto norm4 = [](const std::array<double, 4>& a) -> double { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]); };

  double y_scalar = 0.0;

  if(m_DSType == "VMF")
  {
    // Weighted sum of quaternions rotated into a common frame via symmetry conjugates
    std::array<double, 4> tmpGamma{0, 0, 0, 0};

    for(int j = 0; j < Pmdims; ++j)
    {
      const QuatD symj_conj = m_LaueOps->getQuatSymOp(j).conjugate();
      const size_t colBase = static_cast<size_t>(j) * N;

      for(int i = 0; i < N; ++i)
      {
        const double rij = R[colBase + i];
        QuatD qu = m_XQuats[i] * symj_conj;

        tmpGamma[0] += rij * qu.x();
        tmpGamma[1] += rij * qu.y();
        tmpGamma[2] += rij * qu.z();
        tmpGamma[3] += rij * qu.w();
      }
    }

    const double nGamma = norm4(tmpGamma);
    if(nGamma > 0.0)
    {
      MuKa[0] = tmpGamma[0] / nGamma;
      MuKa[1] = tmpGamma[1] / nGamma;
      MuKa[2] = tmpGamma[2] / nGamma;
      MuKa[3] = tmpGamma[3] / nGamma;
    }
    else
    {
      MuKa[3] = 1.0; // identity quaternion fallback
    }

    y_scalar = nGamma / static_cast<double>(N);
  }
  else if(m_DSType == "WAT")
  {
    // Build scatter matrix Tscatt = (1/N) * sum_{j,i} R(i,j) * (x x^T)
    Eigen::Matrix4d Tscatt = Eigen::Matrix4d::Zero();

    for(int j = 0; j < Pmdims; ++j)
    {
      const QuatD symj_conj = m_LaueOps->getQuatSymOp(j).conjugate();
      const size_t colBase = static_cast<size_t>(j) * N;

      for(int i = 0; i < N; ++i)
      {
        const double rij = R[colBase + i];
        QuatD qu = m_XQuats[i] * symj_conj;

        Eigen::Vector4d xwxyz;
        xwxyz << qu.w(), qu.x(), qu.y(), qu.z();

        Tscatt.noalias() += rij * (xwxyz * xwxyz.transpose());
      }
    }

    if(N > 0)
    {
      Tscatt *= (1.0 / static_cast<double>(N));
    }

    // Dominant eigenvector of the symmetric scatter matrix gives the mean direction
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(Tscatt);
    const Eigen::Vector4d qq = es.eigenvectors().col(3);

    // qq is in (w,x,y,z) order; store in EbsdLib (x,y,z,w) order
    MuKa[0] = qq(1);
    MuKa[1] = qq(2);
    MuKa[2] = qq(3);
    MuKa[3] = qq(0);

    y_scalar = (qq.transpose() * Tscatt * qq)(0, 0);
  }

  // Convert y_scalar -> kappa
  if(y_scalar >= 0.94)
  {
    if(m_DSType == "WAT")
    {
      MuKa[4] = (5.0 * y_scalar - 11.0 - std::sqrt(39.0 - 12.0 * y_scalar + 9.0 * y_scalar * y_scalar)) / (8.0 * (y_scalar - 1.0));
    }
    else
    {
      MuKa[4] = (15.0 - 3.0 * y_scalar + std::sqrt(15.0 + 90.0 * y_scalar + 39.0 * y_scalar * y_scalar)) / (16.0 * (1.0 - y_scalar));
    }
  }
  else
  {
    // Lookup table: find the xAp value whose yAp is closest to y_scalar
    int idx = 0;
    double best = std::numeric_limits<double>::infinity();
    for(int k = 0; k < m_ApNum; ++k)
    {
      const double d = std::abs(y_scalar - m_YAp[k]);
      if(d < best)
      {
        best = d;
        idx = k;
      }
    }
    if(idx == 0 && m_ApNum > 1)
    {
      idx = 1;
    }
    MuKa[4] = m_XAp[idx];
  }

  return MuKa;
}

// Computes the Q-function (expected complete-data log-likelihood) and
// the observed-data log-likelihood L, given current parameters and responsibilities.
void DirectionalStats::getQandL_(const std::array<double, 5>& MuKa, const std::vector<double>& R, double& Q, double& L) const
{
  const int N = this->getN();
  const int Pmdims = m_LaueOps->getNumSymOps();

  const double oldQ = Q;
  const double oldL = L;

  // For VMF, the normalization constant is exponentiated before passing to Density_
  double C = this->logCp_(MuKa[4]);
  if(m_DSType != "WAT")
  {
    C = std::exp(C);
  }

  const QuatD qu(MuKa[0], MuKa[1], MuKa[2], MuKa[3]);

  std::vector<double> phiStorage(static_cast<size_t>(N) * Pmdims, 0.0);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Phi(phiStorage.data(), N, Pmdims);

  for(int j = 0; j < Pmdims; ++j)
  {
    QuatD PmMu = m_LaueOps->getQuatSymOp(j) * qu;

    std::vector<double> col = this->Density_(PmMu, MuKa[4], C);
    if(static_cast<int>(col.size()) != N)
    {
      col.resize(N, 0.0);
    }

    for(int i = 0; i < N; ++i)
    {
      Phi(i, j) = col[i];
    }
  }

  if(Pmdims > 0)
  {
    Phi.array() /= static_cast<double>(Pmdims);
  }

  const double minPhi = Phi.minCoeff();
  if(minPhi > 0.0)
  {
    Eigen::VectorXd rowSums = Phi.rowwise().sum();
    L = rowSums.array().log().sum();

    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Rm(R.data(), N, Pmdims);
    Q = (Rm.array() * Phi.array().log()).sum();
  }
  else
  {
    L = oldL;
    Q = oldQ;
  }
}
