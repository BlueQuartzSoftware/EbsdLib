/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                                 *
 * Copyright (c) 2017, William C. Lenthe                                           *
 * All rights reserved.                                                            *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *                                                                                 *
 * 1. Redistributions of source code must retain the above copyright notice, this  *
 *    list of conditions and the following disclaimer.                             *
 *                                                                                 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,    *
 *    this list of conditions and the following disclaimer in the documentation    *
 *    and/or other materials provided with the distribution.                       *
 *                                                                                 *
 * 3. Neither the name of the copyright holder nor the names of its                *
 *    contributors may be used to endorse or promote products derived from         *
 *    this software without specific prior written permission.                     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"     *
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE       *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  *
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE    *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL      *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR      *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,   *
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.            *
 *                                                                                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//orientation coloring routines based on
// -Nolze, Gert and Hielscher Ralf. "Orientations Perfectly Colors." J. Appl. Crystallogr. 49.5 (2016): 1786-1802.
// -matlab implementation of routines by Ralf Hielscher (https://github.com/mtex-toolbox/mtex)

#ifndef _coloring_h_
#define _coloring_h_

#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>

namespace coloring {
	template <typename T>
	inline void hsv2rgb(T const * const hsv, T * const rgb) {
		const T c = hsv[1] * hsv[2];
		const T x = c * (T(1) - fabs(fmod(T(6) * hsv[0], T(2)) - T(1)));
		const T d = hsv[2] - c;
		switch(((int) std::floor(T(6) * hsv[0])) % 6) {
			case 0:
				rgb[0] = c;
				rgb[1] = x;
				rgb[2] = T(0);
				break;
			case 1:
				rgb[0] = x;
				rgb[1] = c;
				rgb[2] = T(0);
				break;
			case 2:
				rgb[0] = T(0);
				rgb[1] = c;
				rgb[2] = x;
				break;
			case 3:
				rgb[0] = T(0);
				rgb[1] = x;
				rgb[2] = c;
				break;
			case 4:
				rgb[0] = x;
				rgb[1] = T(0);
				rgb[2] = c;
				break;
			case 5:
				rgb[0] = c;
				rgb[1] = T(0);
				rgb[2] = x;
				break;
		}
		std::for_each(rgb, rgb+3, [d](T& i){i += d;});
		std::for_each(rgb, rgb+3, [](T& i){
			if(i > T(1)) i = T(1);
			else if(i < T(0)) i = T(0);});
	}

	template <typename T>
	inline void hsl2hsv(T const * const hsl, T * const hsv) {
		const T l = hsl[2];
		const T s = hsl[1] * (hsl[2] < T(0.5) ? hsl[2] : T(1) - hsl[2]);
		hsv[0] = hsl[0];
		hsv[2] = s + hsl[2];
		hsv[1] = hsv[2] > T(0) ? T(2) * s / hsv[2] : T(0);
	}

	template <typename T>
	inline void hsl2rgb(T const * const hsl, T * const rgb) {hsl2hsv(hsl, rgb); hsv2rgb(rgb, rgb);}

	namespace ipf {
		template <typename T>
		struct Constants {
			static const T pi;
			static const T pi2;
			static const T r2;
		};

		template <typename T> const T Constants<T>::pi      = T(2) * std::acos(T(0));
		template <typename T> const T Constants<T>::pi2     = T(4) * std::acos(T(0));
		template <typename T> const T Constants<T>::r2      = std::sqrt(T(2));

		//math helpers
		template <typename T>
		void unitCartesianToSpherical(T const * const n, T& theta, T& phi) {
			theta = atan2(n[1], n[0]);
			if(theta < T(0)) theta += Constants<T>::pi2;
			phi = std::acos(n[2]);
		}

		template <typename T>
		void sphericalToUnitCartesian(const T theta, const T phi, T * const n) {
			const T s = std::sin(phi);
			n[0] = std::cos(theta) * s;
			n[1] = std::sin(theta) * s;
			n[2] = std::cos(phi);
		}

		template <typename T>
		void cross(T const * const v1, T const * const v2, T * const x) {
			x[0] = v1[1] * v2[2] - v1[2] * v2[1];
			x[1] = v1[2] * v2[0] - v1[0] * v2[2];
			x[2] = v1[0] * v2[1] - v1[1] * v2[0];
		}

		//move to dihedral triangle, returns true/false if actually inside / reflected inside
		template <typename T, size_t N>
		bool cyclicTriangle(T const * const n, T * const nTri) {//returns true/false if inside/outside corresponding dihedral triangle
			//convert to spherical coordinates
			T theta, phi;
			unitCartesianToSpherical(n, theta, phi);

			//bring to cyclic sterographic triangle
			bool dihedral = true;
			theta = std::fmod(theta, Constants<T>::pi2 / N);

			//bring to dihedral sterographic triangle
			if(theta > Constants<T>::pi / N) {
				theta = Constants<T>::pi2 / N - theta;
				dihedral = false;
			}

			//bring to northern hemisphere
			if(n[2] < T(0)) {
				phi = Constants<T>::pi - phi;
				dihedral = !dihedral;

				if(1 == N % 2) {
					theta = Constants<T>::pi / N - theta;
				}
			}

			//convert back to cartesian coordinates
			sphericalToUnitCartesian(theta, phi, nTri);
			return dihedral;
		}

		template <typename T, size_t N>
		void dihedralTriangle(T const * const n, T * const nTri) {
			//move to northern hemisphere
			std::copy(n, n+3, nTri);
			if(nTri[2] < T(0)) std::transform(nTri, nTri+3, nTri, std::negate<T>());

			//convert to spherical coordinates
			T theta, phi;
			unitCartesianToSpherical(nTri, theta, phi);

			//bring to sterographic triangle
			theta = std::fmod(theta, Constants<T>::pi2 / N);
			if(theta > Constants<T>::pi / N) theta = Constants<T>::pi2 / N - theta;
			phi = std::fabs(phi);

			//convert back to cartesian coordinates
			sphericalToUnitCartesian(theta, phi, nTri);
		}

		template <typename T>
		bool cubicLowTriangle(T const * const n, T * const nTri) {
			std::transform(n, n+3, nTri, (T(*)(T))&std::fabs);
			if(nTri[0] >= nTri[1]) {
				if(nTri[0] > nTri[2]) std::rotate(nTri, nTri+1, nTri+3);
			} else {
				if(nTri[1] > nTri[2]) std::rotate(nTri, nTri+2, nTri+3);
			}
			if(nTri[1] > nTri[0]) {
				std::swap(nTri[0], nTri[1]);
				return false;
			}
			return true;
		}

		template <typename T>
		void cubicTriangle(T const * const n, T * const nTri) {
			std::transform(n, n+3, nTri, (T(*)(T))&std::fabs);
			std::sort(nTri, nTri + 3);
			std::swap(nTri[0], nTri[1]);
		}

		//convert a unit direction in a fundamental sector to fractional (0-1) polar coordinates on the northern hemisphere
		template <typename T>
		void fundToHemi(T const * const n,            //unit direction to color
		                T& theta,                     //distance from north pole
		                T& rho,                       //polar angle
		                T const center[3],            //unit direction of fundamental sector center
		                T const normals[3][3],        //unit directions of 3 plane normals defining fundamental sector boundary
		                T const rx[3],                //direction of red from center
		                T const ry[3],                //direction perpendicular to rx and center
		                std::vector<T> const& irho,   //x axis of interpolation array for hue correction
		                std::vector<T> const& omega) {//y axis of interpolation array for hue correction
			//get plane defined by center + direction
			T vxc[3];
			cross(n, center, vxc);
			T mag = std::sqrt(std::inner_product(vxc, vxc+3, vxc, T(0)));
			std::for_each(vxc, vxc+3, [mag](T& i){i /= mag;});

			//compute distance to each edge
			T rMin(1), v[3];
			for(size_t i = 0; i < 3; i++) {
				cross(vxc, normals[i], v);
				mag = std::sqrt(std::inner_product(v, v+3, v, T(0)));
				std::for_each(v, v+3, [mag](T& i){i /= mag;});
				T r = std::acos(-std::inner_product(n, n+3, v, T(0))) / std::acos(-std::inner_product(center, center+3, v, T(0)));
				if(r < rMin) rMin = r;
			}
			theta = T(1) - rMin;

			//compute angle from red direction
			std::transform(n, n + 3, center, v, std::minus<T>());
			mag = std::sqrt(std::inner_product(v, v+3, v, T(0)));
			std::for_each(v, v+3, [mag](T& i){i /= mag;});
			rho = std::atan2(std::inner_product(ry, ry+3, v, T(0)), std::inner_product(rx, rx+3, v, T(0))) / Constants<T>::pi2;
			if(rho < 0.0) rho += T(1);

			//apply adaptive hue gradient + make p001,100,v] [r,g,b]
			if(!(rho <= irho.front() || rho >= irho.back())) {
				size_t index = std::distance(irho.begin(), std::upper_bound(irho.begin(), irho.end(), rho));
				rho = omega[index-1] + ( (irho[index] - rho) / (irho[index] - irho[index - 1]) ) * (omega[index] - omega[index - 1]);
			}
		}

		//convert a unit direction in a dihedral fundamental sector to fractional (0-1) polar coordinates on the northern hemisphere
		template <typename T, size_t N>
		void dihedralToHemi(T const * const n, T& theta, T& rho) {
			//compute constants on first execution
			static_assert(N == 2 || N == 3 || N == 4 || N == 6, "dihedral sector -> hemisphere mapping is only defined for N = 2, 3, 4, or 6");
			static const T angle = T(2) * std::acos(T(0)) / T(N);//pi/N
			static const T s = std::sin(angle);
			static const T c = std::cos(angle);
			static const T c2_3 = T(3) + T(2) * c;
			static const T kc = T(1) / std::sqrt(c2_3);
			static const T kt = std::tan(angle / T(2));
			static const T krx = std::sqrt( T(1) - T(1) / c2_3);
			static const T kry = std::fabs( std::cos(angle / T(2)) );

			static const T center[3] = {( T(1) + c) * kc,  s * kc         , kc  };//barycenter of fundamental sector
			static const T rx[3]     = {-krx / T(2)     , -krx * kt / T(2), krx };//normal of cutting plane that isn't +z or +y
			static const T ry[3]     = { kry * kt       , -kry            , T(0)};//normal of cutting plane that isn't +z or +y
			static const T normals[3][3] = {
				{T(0), T(1), T(0)},//bottom boundary
				{T(0), T(0), T(1)},//right boundary
				{  s ,  -c , T(0)} //'left' boundary
			};
			
			//build lookup table for distance correction to rho
			static std::vector<T> irho, omega;
			if(omega.empty()) {
				//create evenly spaced list for angle from 0->1
				omega.resize(1000);
				irho.resize(omega.size());
				std::iota(irho.begin(), irho.end(), T(0));
				std::for_each(irho.begin(), irho.end(), [](T&i){i /= T(irho.size() - 1);});
				const T rhoG = std::fmod(std::atan2( T(2) * kry * kt, -std::sqrt( T(1) - T(1) / c2_3)) + Constants<T>::pi2, Constants<T>::pi2) / Constants<T>::pi2;//angle of 100
				const T rhoB = std::fmod(std::atan2(-T(2) * kry * kt, -std::sqrt( T(1) - T(1) / c2_3)) + Constants<T>::pi2, Constants<T>::pi2) / Constants<T>::pi2;//angle of vertex of fundamental sector that isn't +z or +x

				//compute distance to edge at each engle
				for(size_t i = 0; i < omega.size() - 1; i++) {
					//create vector normal to center at angle irho[i]
					T n[3];
					T sn = std::sin(Constants<T>::pi2 * irho[i]);
					T cs = std::cos(Constants<T>::pi2 * irho[i]);
					std::transform(rx, rx+3, ry, n, [sn, cs](T i, T j){return i * sn - j * cs;});

					if(irho[i] < rhoG) {//bottom is closest edge (+y cutting plane)
						omega[i+1] = std::acos(( n[2] * center[0] - n[0] * center[2]) / std::hypot(n[0], n[2]));
					} else if(irho[i] < rhoB) {//right is cosest edge (+z cutting plane)
						omega[i+1] = std::acos((-n[1] * center[0] + n[0] * center[1]) / std::hypot(n[1], n[0]));
					} else {
						T normxn[3];
						cross(normals[2], n, normxn);
						T mag = std::sqrt(std::inner_product(normxn, normxn+3, normxn, T(0)));
						omega[i+1] = std::acos(std::inner_product(normxn, normxn+3, center, T(0)) / mag);
					}
				}

				//get offset to green and blue points
				const size_t indG = std::distance(irho.begin(), std::upper_bound(irho.begin(), irho.end(), rhoG));
				const size_t indB = std::distance(irho.begin(), std::upper_bound(irho.begin(), irho.end(), rhoB));

				//normalize
				const T sumRG = T(3) * std::accumulate(omega.begin()       , omega.begin() + indG, T(0));
				const T sumGB = T(3) * std::accumulate(omega.begin() + indG, omega.begin() + indB, T(0));
				const T sumBR = T(3) * std::accumulate(omega.begin() + indB, omega.end()         , T(0));

				std::for_each(omega.begin()       , omega.begin() + indG, [sumRG](T& i){i /= sumRG;});
				std::for_each(omega.begin() + indG, omega.begin() + indB, [sumGB](T& i){i /= sumGB;});
				std::for_each(omega.begin() + indB, omega.end()         , [sumBR](T& i){i /= sumBR;});

				//integrate
				std::partial_sum(omega.begin(), omega.end(), omega.begin());
			}
			fundToHemi(n, theta, rho, center, normals, rx, ry, irho, omega);
		}

		//convert a unit direction in the cubic fundamental sector to fractional (0-1) polar coordinates on the northern hemisphere
		template <typename T>
		void cubicToHemi(T const * const n, T& theta, T& rho) {
			//analytic forms exist for these but are pretty ugly
			static const T center[3]     = {T( 0.47862549063280972775795557014085), T( 0.21513724867401406276755961370160), T(0.85125413593678216086647016667920)};
			static const T rx[3]         = {T(-0.77642514034632512230434735649784), T(-0.34899513662466263468169382471815), T(0.52475365272718435652736178596868)};
			static const T ry[3]         = {T( 0.40997761055293190765064079176041), T(-0.91209558646301347822616475798505), T(0.00000000000000000000000000000000)};
			static const T normals[3][3] = {
				{ T(0)                  ,  T(1)                  , T(0)                  },//bottom boundary
				{-T(1) / std::sqrt(T(2)), T(0)                   , T(1) / std::sqrt(T(2))},//right boundary
				{ T(1) / std::sqrt(T(2)), -T(1) / std::sqrt(T(2)), T(0)                  } //top boundary
			};

			//build lookup table for nonlinear hue adjustment
			static std::vector<T> irho, omega;
			if(omega.empty()) {
				//create evenly spaced list for angle from 0->1
				omega.resize(1000);
				irho.resize(omega.size());
				std::iota(irho.begin(), irho.end(), T(0));
				std::for_each(irho.begin(), irho.end(), [](T&i){i /= T(irho.size() - 1);});

				//compute distance to edge at each engle
				const T rhoG = T(0.33762324015537352214801096852002);
				const T rhoB = T(0.61081504295610182824357048263158);
				for(size_t i = 0; i < omega.size() - 1; i++) {
					//create vector normal to center at angle irho[i]
					T n[3];
					T s = std::sin(Constants<T>::pi2 * irho[i]);
					T c = std::cos(Constants<T>::pi2 * irho[i]);
					std::transform(rx, rx+3, ry, n, [s, c](T i, T j){return i * s - j * c;});

					if(irho[i] < rhoG) {//bottom is closest edge
						T mag = std::hypot(n[2], n[0]);
						omega[i+1] = std::acos((center[0] * n[2] - center[2] * n[0]) / mag );
					} else if(irho[i] < rhoB) {//right is cosest edge)
						T mag = std::hypot(n[1], (n[0] + n[2]) / Constants<T>::r2 ) * Constants<T>::r2;
						omega[i+1] = std::acos(-( ( center[0] +  center[2] ) * n[1] - center[1] * ( n[0] + n[2] ) ) / mag );
					} else {//left is closest edge
						T mag = std::hypot(n[2], (n[1] + n[0]) / Constants<T>::r2 ) * Constants<T>::r2;
						omega[i+1] = std::acos(-( ( center[1] +  center[0] ) * n[2] - center[2] * ( n[1] + n[0] ) ) / mag );
					}
				}

				//get offset to green and blue points
				const size_t indG = std::distance(irho.begin(), std::upper_bound(irho.begin(), irho.end(), rhoG));
				const size_t indB = std::distance(irho.begin(), std::upper_bound(irho.begin(), irho.end(), rhoB));

				//normalize
				const T sumRG = T(3) * std::accumulate(omega.begin()       , omega.begin() + indG, T(0));
				const T sumGB = T(3) * std::accumulate(omega.begin() + indG, omega.begin() + indB, T(0));
				const T sumBR = T(3) * std::accumulate(omega.begin() + indB, omega.end()         , T(0));

				std::for_each(omega.begin()       , omega.begin() + indG, [sumRG](T& i){i /= sumRG;});
				std::for_each(omega.begin() + indG, omega.begin() + indB, [sumGB](T& i){i /= sumGB;});
				std::for_each(omega.begin() + indB, omega.end()         , [sumBR](T& i){i /= sumBR;});

				//integrate
				std::partial_sum(omega.begin(), omega.end(), omega.begin());
			}
			fundToHemi(n, theta, rho, center, normals, rx, ry, irho, omega);
		}

		//convert fractional (0-1) polar coordinates on the northern hemisphere to fractional rgb
		template <typename T>
		void hemiToRgb(T theta, const T rho, T * const rgb, bool whiteCenter = true) {
			const T p = whiteCenter ? T(1) - theta / T(2) : theta / T(2);
			const T yL = whiteCenter ? T(0.25) : T(0.5);
			const T yS = whiteCenter ? T(0.2 ) : T(0.5);

			//constants for nonlinear hue adjustment
			static const T denom = T(1) + std::sqrt(T(2) * Constants<T>::pi) * std::erf( T(5) * std::sqrt(T(2)) / T(3) ) * T(3) / T(10);
			static const T k1 = std::sqrt( Constants<T>::pi / T(2) ) / T(10);
			static const T k2 = T(10) * std::sqrt(T(2));
			static const T k1_3 = T(1) / T(3);
			static const T k1_6 = T(1) / T(6);

			//adjust hue gradient
			T hsl[3];
			const T h3 = std::fmod(rho, k1_3);
			const bool half = h3 > k1_6;
			const T h6 = half ? k1_3 - h3 : h3;
			const T hNew = (h6 + k1 * std::erf(k2 * h6)) / denom;
			hsl[0] = rho - h3 + (half ? k1_3 - hNew : hNew);

			//adjust lightness gradient
			const T sP = std::sin(p * Constants<T>::pi / T(2));
			const T th = yL * p + (T(1) - yL) * sP * sP;
			const T gray = T(1) - T(2) * yS * std::fabs(th - T(0.5));
			hsl[2] = (th - T(0.5)) * gray + T(0.5);

			//adjust saturation gradient
			hsl[1] = gray * ( T(1) - std::fabs( T(2) * th - T(1) ) ) / ( T(1) - std::fabs( T(2) * hsl[2] - T(1) ) );
			if(std::isnan(hsl[1])) hsl[1] = T(0);

			//convert to rgb
			hsl2rgb(hsl, rgb);
		}

	}

	//-1
	template <typename T>
	void hemiIpf(T const * const n, T * const rgb) {
		//convert to fractional spherical coordinates
		T theta, phi;
		ipf::unitCartesianToSpherical(n, theta, phi);
		theta /= ipf::Constants<T>::pi2;
		phi /= ipf::Constants<T>::pi;

		//move to northern hemisphere
		bool whiteCenter = true;
		if(phi > T(1) / T(2)) {
			phi = T(1) - phi;
			whiteCenter = false;
		}
		phi *= T(2);

		//convert to rgb
		ipf::hemiToRgb(phi, theta, rgb, whiteCenter);
	}

	//222, -3, 422, or 622
	template <typename T, size_t N>
	void cyclicIpf(T const * const n, T * const rgb) {
		T nFs[3], theta, rho;
		bool whiteCenter = ipf::cyclicTriangle<T, N>(n, nFs);//move to fundamental sector
		ipf::dihedralToHemi<T, N>(nFs, theta, rho);//stretch to northern hemisphere
		ipf::hemiToRgb(theta, rho, rgb, whiteCenter);//convert to rgb
	}

	//mmm, -3m, 4/mmm, or 6/mmm
	template <typename T, size_t N>
	void dihedralIpf(T const * const n, T * const rgb) {
		T nFs[3], theta, rho;
		bool whiteCenter = true;
		if(3 == N) {//handle -3m
			whiteCenter = ipf::cyclicTriangle<T, 3>(n, nFs);//check if white/black center
			ipf::dihedralTriangle<T, 6>(n, nFs);//move to fundamental sector
			ipf::dihedralToHemi<T, 6>(nFs, theta, rho);//stretch to northern hemisphere
		} else {
			ipf::dihedralTriangle<T, N>(n, nFs);//move to fundamental sector
			ipf::dihedralToHemi<T, N>(nFs, theta, rho);//stretch to northern hemisphere
		}
		ipf::hemiToRgb(theta, rho, rgb, whiteCenter);//convert to rgb
	}
	
	//m-3
	template <typename T>
	void cubicLowIpf(T const * const n, T * const rgb) {
		T nFs[3], theta, rho;
		bool whiteCenter = ipf::cubicLowTriangle(n, nFs);//move to fundamental sector
		ipf::cubicToHemi(nFs, theta, rho);//stretch to northern hemisphere
		ipf::hemiToRgb(theta, rho, rgb, whiteCenter);//convert to rgb
	}

	//m-3m
	template <typename T>
	void cubicIpf(T const * const n, T * const rgb) {
		T nFs[3], theta, rho;
		ipf::cubicTriangle(n, nFs);//move to fundamental sector
		ipf::cubicToHemi(nFs, theta, rho);//stretch to northern hemisphere
		ipf::hemiToRgb(theta, rho, rgb);//convert to rgb
	}
}

#endif//_coloring_h_