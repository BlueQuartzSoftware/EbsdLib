#include "DirectionalStats.hpp"

#include "EbsdLib/Orientation/Quaternion.hpp"

// ======================= DirectionStats::EMforDS_ (QuatD + OrientationD) =================
//
// Assumptions (adjust names if they differ in your codebase):
//   using QuatD = Quaternion<double>;   // your alias
//   class Quaternion<T> {
//     Quaternion(T x, T y, T z, T w);   // (x,y,z,w)
//     void normalize();
//     void makePositive();               // canonicalize sign like Fortran's quat_pos()
//     Quaternion operator*(const Quaternion&) const;
//   };
//
//   class OrientationD {                 // Rodrigues-like; ctor: (x, y, z, l)
//     OrientationD(double x, double y, double z, double l);
//   };
//
//   class Symmetry {
//     int getQnumber() const;
//     QuatD getQuatfromArray(int one_based_index) const; // Fortran-style indexing
//   };
//
//   class DirectionStats {
//     int getN() const;
//     int NumEM, NumIter, pgnum;
//     Symmetry qsym;
//     std::vector<double> Estep_(const QuatD& Mu, double Kappa) const; // size N*Pmdims
//     // Returns [w,x,y,z,kappa] in Fortran order:
//     std::array<double,5> Mstep_(const std::vector<double>& R, int N, int Pmdims) const;
//     void getQandL_(const std::array<double,5>& MuKa,
//                    const std::vector<double>& R,
//                    double& Qout, double& Lout) const;
//   };

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
  const T a = q1 + y * (q2 + y * (q3 + y * (q4 + y * (q5 + y * (q6 + y * (q7 + y * (q8 + y * 19)))))));
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
  if(N == 0) {
    return BesselI0(x);
}
  if(N == 1) {
    return BesselI1(x);
}
  if(x == T(0)) {
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
    if(j == N) {
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
  if(s < 0) {
    s += 2147483647;
}
  seed = static_cast<uint32_t>(s);
  return static_cast<double>(s) * 4.656612875e-10;
}

void r8vec_uniform_01(int m, uint32_t& seed, std::vector<double>& r)
{
  r.resize(m);
  for(int i = 0; i < m; ++i) {
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
    if(r1 <= 0.0) {
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
: DStype(DSType)
{
  m_LaueOps = laueOps;

  // von Mises-Fisher mode: (DStype='VMF')
  // the next part of the initial Matlab code computes a lookup table for the parameter Ap(u) (Appendix in paper)
  // this lookup table is only used when the ratio of the BesselI functions is between 0 and 0.95; for the
  // region between 0.95 and 1, we use an analytical approximation (see VMF_Mstep routine).
  //
  // Watson mode: (DStype='WAT')
  // we've used a similar approach to create a lookup table for values of kappa that are smaller than 35, in
  // which case we use the standard ratio of Kummer functions:  Kummer[3/2,3,k]/Kummer[1/2,2,k]/k.  For
  // larger kappa values, we have an expansion using the large argument behavior of the modified Bessel functions.
  //

  // ----- Optional DStype setup and lookup table generation -----
  if(!DSType.empty())
  {
    // this->DStype = DStype;

    // Allocate/size parameter arrays
    this->Apnum = 35000;
    this->xAp.resize(this->Apnum);
    this->yAp.resize(this->Apnum);

    // Define xAp(i) = 0.001 + (i-1)*0.001, i=1..Apnum  (Fortran 1-based)
    // In 0-based C++: xAp[k] = 0.001 + k*0.001
    for(int k = 0; k < this->Apnum; ++k)
    {
      this->xAp[k] = 0.001 + static_cast<double>(k) * 0.001;
    }

    if(this->DStype == "VMF")
    {
      // yAp(i) = I2(x) / I1(x)
      for(int k = 0; k < this->Apnum; ++k)
      {
        const double x = this->xAp[k];
        const double denom = static_cast<double>(BesselI1(x));
        const double numer = static_cast<double>(BesselI2(x));
        // Guard against zero denom (very small x)
        const double safeDen = (std::abs(denom) > 1e-300) ? denom : std::numeric_limits<double>::min();
        this->yAp[k] = numer / safeDen;
      }
    }
    else if(this->DStype == "WAT")
    {
      // yAp(i) = I1(x/2) / ((I0(x/2) - I1(x/2)) * x)
      for(int k = 0; k < this->Apnum; ++k)
      {
        const double x = this->xAp[k];
        const double xh = 0.5 * x;
        const double I1h = static_cast<double>(BesselI1(xh));
        const double I0h = static_cast<double>(BesselI0(xh));
        double denom = (I0h - I1h) * x;
        if(!(std::abs(denom) > 0.0))
          denom = std::numeric_limits<double>::min(); // guard
        this->yAp[k] = I1h / denom;
      }
    }
  }

  // ----- Optional symmetry initialization -----
  // if (PGnumOpt >= 0)
  //   {
  //   this->pgnum = PGnumOpt;
  //   this->qsym.QSym_Init(this->pgnum);
  // this->Pmdims_ = m_LaueOps->getNumSymOps();
  // }
}

DirectionalStats::~DirectionalStats() = default;

// author: MDG, based on 2015 Chen's Matlab code, with simplifications
// version: 1.0
// date: 01/23/20
//
// Expectation maximization approach to maximum likelihood problem for mu and kappa
//
// this routine expects the input quaternion array to be stored in Xquats using the setQuatArray method
void DirectionalStats::EMforDS(uint32_t& seed, QuatD& muhat, double& kappahat, bool verbose)
{
  // In this routine, we perform the EM algorithm to obtain an estimate for the
  // mean direction and concentration parameter of the modified von Mises-Fisher (mVMF)
  // distribution that models the statistics of the orientation point cloud.

  // array sizes
  const int N = this->getN();
  const int pmdims = m_LaueOps->getNumSymOps();
  const int numEm = this->NumEM_;
  const int numIter = this->NumIter_;

  // initialize some auxiliary arrays
  std::vector<QuatD> muAll(numEm); //
  std::vector<double> kappaAll(numEm, 0.0);
  std::vector<double> lAll(numEm, 0.0);

  // auto idxMu = [&](int init, int k) { return init * 4 + k; }; // k=0..3 → (w,x,y,z)
  // main loop (EM typically uses a few starting parameter sets to make sure we don't get stuck in a local maximum)
  for(int init = 0; init < numEm; ++init)
  {
    // generate a normal random vector and normalize it as a starting guess for Mu (i.e., a unit quaternion)
    std::array<double, 4> v;
    r8vec_normal_01(4, seed, v.data());
    // v comes from Fortran-order PRNG: v[0]=w, v[1]=x, v[2]=y, v[3]=z
    QuatD mu = QuatD(v[1], v[2], v[3], v[0]).normalize().getPositiveOrientation();

    // starting value for Kappa
    double kappa = 30.0;
    // define the number of iterations and the Q and L function arrays
    std::vector<double> q(numIter, 0.0);
    std::vector<double> l(numIter, 0.0);

    // and here we go with the EM iteration...
    // we use quaternion multiplication throughout instead of the matrix version in the Matlab version
    // quaternion multiplication has been verified against the 4x4 matrix multiplication of the Matlab code on 01/02/15
    for(int i = 0; i < numIter; ++i)
    {
      // E-step
      std::vector<double> const r = this->Estep_(mu, kappa);

      // M-step — returns MuKa
      std::array<double, 5> muKa = this->Mstep_(r, N, pmdims);

      // Q and Likelihood
      double Qi = 0.0, Li = 0.0;
      this->getQandL_(muKa, r, Qi, Li);
      q[i] = Qi;
      l[i] = Li;

      // Persist latest params for this init
      // MuKa is [x,y,z,w,kappa] matching QuatD(x,y,z,w) constructor
      muAll[init] = QuatD(muKa[0], muKa[1], muKa[2], muKa[3]);
      kappaAll[init] = muKa[4];
      lAll[init] = l[i];

      // Update Mu/Kappa for next iter (Fortran does NOT call quat_pos here)
      mu = muAll[init];
      kappa = kappaAll[init];

      // Convergence: |Q(i) - Q(i-1)| < 0.01
      if(i >= 1 && std::fabs(q[i] - q[i - 1]) < 0.01)
      {
        break;
      }
    }
  }

  // Pick best init by max likelihood
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

  // Recover Mu for best init
  QuatD mu = muAll[dd];
  mu.positiveOrientation();
  kappahat = kappaAll[dd];

  // Ensure Mu lies in the fundamental zone:
  // Cycle symmetry equivalents (Fortran loop i=1..Pmdims → C++ i=0..Pmdims-1 with +1)
  QuatD const quat = mu;
  for(int i = 0; i < pmdims; ++i)
  {
    QuatD const qi = m_LaueOps->getQuatSymOp(i); // 1-based access
    QuatD const qu = (quat * qi).getPositiveOrientation();

    // test FZ, and, if inside, convert back
    if(m_LaueOps->IsInsideFZ(qu, m_LaueOps->getFZType(), m_LaueOps->getAxisOrderingType()))
    {
      muhat = qu;
      return;
    }
  }

  // Fallback (once stubs are real, we should have returned above)
  muhat = mu;
  muhat.positiveOrientation();
}

// Computes the E-step responsibilities matrix R (size N x Pmdims), column-major.
// Fortran reference:
//   C = self%logCp_(kappa)
//   do j=1,self%Pmdims
//     PmMu = Mu * self%qsym%getQuatfromArray(j)
//     R(1:self%N,j) = self%Density_(PmMu%get_quatd(), Kappa, C)
//   end do
//   Rdenom = 1.D0/sum(R,2)    // row-wise sum over columns
//   do j=1,self%Pmdims
//     R(1:self%N,j) = R(1:self%N,j)*Rdenom(1:self%N)
//   end do
//
// Assumed DirectionStats API (adjust if names differ):
//   int getN() const;
//   Symmetry qsym;                        // qsym.getQuatfromArray(1..Pmdims)
//   int Pmdims == qsym.getQnumber()
//   double logCp_(double kappa) const;
//   std::vector<double> Density_(const QuatD& q, double kappa, double C) const; // returns N-length vector
//
// Returns: std::vector<double> of length N*Pmdims, column-major.
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

// Returns y of length N, where
//   VMF: y_j = exp( C + kappa * dot(mu, q_j) )
//   WAT: y_j = exp( C + kappa * dot(mu, q_j)^2 )
//
// Args:
//   mu     : mean direction (QuatD; stored as (x,y,z,w))
//   kappa  : concentration parameter
//   C      : precomputed logCp(kappa) (i.e., log normalization constant)
//
// Notes:
// - Fortran used dot product on (w,x,y,z); here we compute the dot in (x,y,z,w)
//   for both operands consistently, which is equivalent.
std::vector<double> DirectionalStats::Density_(const QuatD& mu, double kappa, double C) const
{
  const int N = this->getN();
  std::vector<double> y(N);

  const bool isVMF = (this->DStype == "VMF");
  const bool isWAT = (this->DStype == "WAT");

  for(int j = 0; j < N; ++j)
  {
    QuatD q = m_XQuats[j];

    const double dp = mu.dotProduct(q);

    if(isVMF)
    {
      y[j] = std::exp(C + kappa * dp);
    }
    else if(isWAT)
    {
      y[j] = std::exp(C + kappa * (dp * dp));
    }
    else
    {
      // Default to VMF if type is unrecognized (you can throw/log if preferred)
      y[j] = std::exp(C + kappa * dp);
    }
  }

  return y;
}

double DirectionalStats::logCp_(double kappa) const
{
  // Precomputed constants (copied verbatim from Fortran)
  // C  = ln(1 / (2*pi)^2)
  // C2 = ln(512 / sqrt(2) / pi^(3/2))
  // C2W = ln(128 * sqrt(pi))
  constexpr double C = -3.675754132818690967;   // ln(1/(2*pi)^2)
  constexpr double C2 = 4.1746562059854348688;  // ln(512/sqrt(2)/pi^(3/2))
  constexpr double C2W = 5.4243952068443172530; // ln(128*sqrt(pi))

  const bool isVMF = (this->DStype == "VMF");
  const bool isWAT = (this->DStype == "WAT");

  double lCp = 0.0;

  if(isVMF)
  {
    // For kappa > 30: approximation
    if(kappa > 30.0)
    {
      // lCp = C2 - kappa + log( kappa^4.5 / (-105 + 8*kappa*(-15 + 16*kappa*(-3 + 8*kappa))) )
      // Compute in log-space for stability:
      const double num_log = 4.5 * std::log(kappa);
      const double den_poly = -105.0 + 8.0 * kappa * (-15.0 + 16.0 * kappa * (-3.0 + 8.0 * kappa));
      lCp = C2 - kappa + (num_log - std::log(std::abs(den_poly)));
    }
    else
    {
      // lCp = C + log( kappa / I1(kappa) )
      const double I1 = BesselI1(kappa);
      const double denom = (I1 > 0.0) ? I1 : std::numeric_limits<double>::min();
      lCp = C + std::log(kappa / denom);
    }
    return lCp;
  }

  if(isWAT)
  {
    if(kappa > 20.0)
    {
      // lCp = C2W - kappa + log( kappa^4.5 / (525 + 4*kappa*(45 + 8*kappa*(3 + 4*kappa))) )
      const double num_log = 4.5 * std::log(kappa);
      const double den_poly = 525.0 + 4.0 * kappa * (45.0 + 8.0 * kappa * (3.0 + 4.0 * kappa));
      lCp = C2W - kappa + (num_log - std::log(den_poly));
    }
    else
    {
      // lCp = -0.5*kappa - log( I0(0.5*kappa) - I1(0.5*kappa) )
      const double x = 0.5 * kappa;
      const double I0 = BesselI0(x);
      const double I1 = BesselI1(x);
      double diff = I0 - I1;
      if(!(diff > 0.0))
        diff = std::numeric_limits<double>::min(); // guard
      lCp = -0.5 * kappa - std::log(diff);
    }
    return lCp;
  }

  // Fallback (if DStype is neither VMF nor WAT): return something sane; VMF default.
  // You might prefer to throw or assert instead.
  const double I1 = BesselI1(kappa);
  const double denom = (I1 > 0.0) ? I1 : std::numeric_limits<double>::min();
  return C + std::log(kappa / denom);
}

std::array<double, 5> DirectionalStats::Mstep_(const std::vector<double>& R, int N, int Pmdims) const
{
  std::array<double, 5> MuKa{0, 0, 0, 0, 0}; // [x,y,z,w,kappa] (EbsdLib order)
  auto norm4 = [](const std::array<double, 4>& a) -> double { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]); };

  double y_scalar = 0.0; // "y" in the Fortran, used to compute kappa at the end

  if(this->DStype == "VMF")
  {
    // ----- VMF branch -----
    // tmpGamma = sum_{j=1..Pmdims} sum_{i=1..N} R(i,j) * (X_i * conj(qsym_j))
    std::array<double, 4> tmpGamma{0, 0, 0, 0}; //

    for(int j = 0; j < Pmdims; ++j)
    {
      const QuatD symj_conj = m_LaueOps->getQuatSymOp(j).conjugate();
      const size_t colBase = static_cast<size_t>(j) * N;

      for(int i = 0; i < N; ++i)
      {
        const double rij = R[colBase + i];
        QuatD xi = this->m_XQuats[i]; //
        QuatD qu = xi * symj_conj;

        // accumulate in EbsdLib (x,y,z,w) order
        tmpGamma[0] += rij * qu.x();
        tmpGamma[1] += rij * qu.y();
        tmpGamma[2] += rij * qu.z();
        tmpGamma[3] += rij * qu.w();
      }
    }

    const double nGamma = norm4(tmpGamma);
    if(nGamma > 0.0)
    {
      MuKa[0] = tmpGamma[0] / nGamma; // x
      MuKa[1] = tmpGamma[1] / nGamma; // y
      MuKa[2] = tmpGamma[2] / nGamma; // z
      MuKa[3] = tmpGamma[3] / nGamma; // w
    }
    else
    {
      MuKa[0] = MuKa[1] = MuKa[2] = 0.0;
      MuKa[3] = 1.0; // fallback: identity quaternion
    }

    y_scalar = nGamma / static_cast<double>(N);

  }
  else if(this->DStype == "WAT")
  {
    // ----- WAT branch -----
    // Build Tscatt = (1/N) * sum_{j,i} R(i,j) * (x x^T), with x in (w,x,y,z)
    Eigen::Matrix4d Tscatt = Eigen::Matrix4d::Zero();

    for(int j = 0; j < Pmdims; ++j)
    {
      const QuatD symj_conj = m_LaueOps->getQuatSymOp(j).conjugate();
      const size_t colBase = static_cast<size_t>(j) * N;

      for(int i = 0; i < N; ++i)
      {
        const double rij = R[colBase + i];
        QuatD xi = m_XQuats[i];
        QuatD qu = xi * symj_conj;

        // x in wxyz
        Eigen::Vector4d xwxyz;
        xwxyz << qu.w(), qu.x(), qu.y(), qu.z();

        Tscatt.noalias() += rij * (xwxyz * xwxyz.transpose());
      }
    }

    if(N > 0)
      Tscatt *= (1.0 / static_cast<double>(N));

    // Largest eigenpair of symmetric Tscatt
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(Tscatt);
    // Eigenvalues are ascending; the dominant eigenvector is the last column
    const Eigen::Vector4d qq = es.eigenvectors().col(3);

    // Mu = dominant eigenvector (Fortran used A(:,4) from DSYEV with UPLO='U')
    // qq is in (w,x,y,z) order from Eigen; store in EbsdLib (x,y,z,w) order
    MuKa[0] = qq(1); // x
    MuKa[1] = qq(2); // y
    MuKa[2] = qq(3); // z
    MuKa[3] = qq(0); // w

    // y = qq^T * Tscatt * qq
    y_scalar = (qq.transpose() * Tscatt * qq)(0, 0);
  }
  else
  {
    // Unknown type conservative fallback: identity quaternion in (x,y,z,w)
    MuKa[0] = MuKa[1] = MuKa[2] = 0.0;
    MuKa[3] = 1.0;
    y_scalar = 1.0 / static_cast<double>(std::max(1, N));
  }

  // ----- Convert y -> kappa (same as Fortran) -----
  if(y_scalar >= 0.94)
  {
    if(this->DStype == "VMF")
    {
      MuKa[4] = (15.0 - 3.0 * y_scalar + std::sqrt(15.0 + 90.0 * y_scalar + 39.0 * y_scalar * y_scalar)) / (16.0 * (1.0 - y_scalar));
    }
    else if(this->DStype == "WAT")
    {
      MuKa[4] = (5.0 * y_scalar - 11.0 - std::sqrt(39.0 - 12.0 * y_scalar + 9.0 * y_scalar * y_scalar)) / (8.0 * (y_scalar - 1.0));
    }
    else
    {
      // default VMF-style
      MuKa[4] = (15.0 - 3.0 * y_scalar + std::sqrt(15.0 + 90.0 * y_scalar + 39.0 * y_scalar * y_scalar)) / (16.0 * (1.0 - y_scalar));
    }
  }
  else
  {
    // Lookup: minloc(|y - yAp|), with the Fortran quirk (if idx==1 → use 2)
    int M = static_cast<int>(this->Apnum); // mirror Fortran's Apnum
    if(M <= 0)
    {
      MuKa[4] = 30.0; // defensive default
    }
    else
    {
      // assume xAp and yAp are sized at least Apnum
      int idx = 0;
      double best = std::numeric_limits<double>::infinity();
      for(int k = 0; k < M; ++k)
      {
        const double d = std::abs(y_scalar - this->yAp[k]);
        if(d < best)
        {
          best = d;
          idx = k;
        }
      }
      if(idx == 0 && M > 1)
        idx = 1; // Fortran: if (minp.eq.1) minp = 2
      MuKa[4] = this->xAp[idx];
    }
  }

  return MuKa;
}

// Computes Q and L given MuKa = [x,y,z,w,kappa] (EbsdLib order)
// and R (responsibilities, N x Pmdims, column-major).
void DirectionalStats::getQandL_(const std::array<double, 5>& MuKa, const std::vector<double>& R, double& Q, double& L) const
{
  const int N = this->getN();
  const int Pmdims = m_LaueOps->getNumSymOps();

  // Keep old values in case Phi has non-positive entries
  const double oldQ = Q;
  const double oldL = L;

  // C = logCp_(kappa); For VMF ONLY, Fortran exponentiates C before passing to Density_
  double C = this->logCp_(MuKa[4]); // MuKa[4] is kappa
  if(this->DStype == "VMF")
  {
    C = std::exp(C);
  }

  // Build Phi(N, Pmdims) column-wise using Density_(PmMu, kappa, C)
  // MuKa is (x,y,z,w) matching QuatD convention
  const QuatD qu(MuKa[0], MuKa[1], MuKa[2], MuKa[3]);

  std::vector<double> Phi_storage(static_cast<size_t>(N) * Pmdims, 0.0);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Phi(Phi_storage.data(), N, Pmdims);

  for(int j = 0; j < Pmdims; ++j)
  {
    // PmMu = qsym(j+1) * qu   (Fortran did: PmMu = qsym(j) * qu)
    QuatD PmMu = m_LaueOps->getQuatSymOp(j) * qu;

    // Density_ returns an N-length vector
    std::vector<double> col = this->Density_(PmMu, MuKa[4], C);
    if(static_cast<int>(col.size()) != N)
      col.resize(N, 0.0);

    // Store as column j (column-major)
    for(int i = 0; i < N; ++i)
    {
      Phi(i, j) = col[i];
    }
  }

  if(Pmdims > 0)
  {
    Phi.array() /= static_cast<double>(Pmdims);
  }

  // If minval(Phi) > 0, compute:
  //   L = sum( log( sum(Phi, 2) ) )     // row-wise sum, then log and sum
  //   Q = sum( R * log(Phi) )           // elementwise
  const double minPhi = Phi.minCoeff();
  if(minPhi > 0.0)
  {
    // L
    Eigen::VectorXd rowSums = Phi.rowwise().sum();
    L = rowSums.array().log().sum();

    // Q
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Rm(R.data(), N, Pmdims);
    Q = (Rm.array() * Phi.array().log()).sum();
  }
  else
  {
    // Reuse old values if any Phi <= 0
    L = oldL;
    Q = oldQ;
  }
}
