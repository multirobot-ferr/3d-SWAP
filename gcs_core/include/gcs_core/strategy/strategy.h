//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 Carmelo J. Fernández-Agüera Tortosa
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
// Strategy testing and simulation environment for mbzirc competition
//----------------------------------------------------------------------------------------------------------------------
#ifndef _MBZIRCUS_STRATEGY_STRATEGY_H_
#define _MBZIRCUS_STRATEGY_STRATEGY_H_

#include <vector>

namespace grvc { namespace mbzirc {
	
	class Agent;
	class ScanPolicy;
	class CarrierPolicy;
	class TargetEstimator;

	class Strategy {
	public:
		Strategy(const std::vector<Agent>& _r, const std::vector<TargetEstimator*>&, ScanPolicy&, CarrierPolicy&);
		/// run strategy
		/// \param _dt time interval in seconds
		void step(float _dt);

	private:
		/// \return true if any observations have located any target directly
		/// \param _located vector to receive estimations of located targets
		/*
		bool observe(Arena*, std::vector<Target::Desc>& _located, float _dt);
		void scan(Arena* _a);
		std::vector<TargetEstimator*>	mEstimators;
		ScanPolicy*						mScanPcy;
		CarrierPolicy*					mCarrierPcy;
		RobotTaskManager				mTaskManager;
		*/
	};
	
}} // namespace grvc::mbzirc

#endif // _MBZIRCUS_STRATEGY_STRATEGY_H_