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
#include <gcs_core/strategy/strategy.h>

namespace grvc { namespace mbzirc {
	//------------------------------------------------------------------------------------------------------------------
	Strategy::Strategy(const std::vector<Agent>& _r, const std::vector<TargetEstimator*>& _e, ScanPolicy* _s, CarrierPolicy* _c)
		:mEstimators(_e)
		,mScanPcy(_s)
		,mCarrierPcy(_c)
		//,mTaskManager(_r)
	{
	}

	//------------------------------------------------------------------------------------------------------------------
	void Strategy::step(double _dt) {
		assert(mCarrierPcy);
		assert(mScanPcy);
		observe(_dt);// Observe and update estimations
		updatePolicies(_dt);// Update policies (policies are responsible for actually sending commands)
	}

	//------------------------------------------------------------------------------------------------------------------
	void Strategy::observe(double _dt) {
		_dt;
	}

	//------------------------------------------------------------------------------------------------------------------
	void Strategy::updatePolicies(double _dt) {
		// Update carrier policy given the latest estimates
		// If something must be carried and there are carriers available, reassign
		// Scan with whatever you have left
	}

	/*
	//------------------------------------------------------------------------------------------------------------------
	void Strategy::step(Arena* _a, float _dt) {
		assert(mCarrierPcy);
		assert(mScanPcy);
		mTaskManager.updateRobotStates(); // Refresh states
		// Observe
		std::vector<Target::Desc> locatedTargets;
		bool newTargetsLocated = observe(_a, locatedTargets, _dt);
		// Assign carriers
		if(newTargetsLocated) {
			std::map<Robot*,Target::Desc>	assigns;
			mCarrierPcy->assignCarriers(mTaskManager.freeForCarry(), locatedTargets, assigns); // Assign carriers
			for (auto a : assigns) {
				mTaskManager.assignToCarry(a.first);
				a.first->captureTarget(a.second);
			}
		}
		// Scan
		scan(_a);
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Strategy::observe(Arena* _a, std::vector<Target::Desc>& _dst, float _dt) {
		// Gather observations
		std::vector<TargetEstimator::Observation> observations(_a->agents().size());
		bool found = false;
		for (unsigned i = 0; i < _a->agents().size(); ++i) {
			auto& o = observations[i];
			auto r = _a->agents()[i];
			o.seenTargets.clear();
			o.area = r->viewArea();
			if(r->scanTerrain(o.seenTargets) > 0) { // target found
				found = true;
				_dst.insert(_dst.end(), o.seenTargets.begin(), o.seenTargets.end());
			}
		}
		// Process observations
		for(auto estimator : mEstimators) {
			for(auto o : observations)
				estimator->processObservation(o);
			// Update estimates
			estimator->updateEstimates(_dt);
		}
		return found;
	}

	//------------------------------------------------------------------------------------------------------------------
	void Strategy::scan(Arena* _a) {
		// Gather free agents
		std::map<Robot*,Vector>	destinations;
		mScanPcy->scanStep(_a, mTaskManager.freeForScan(), destinations);
		for (auto d : destinations) {
			mTaskManager.assignToScan(d.first);
			d.first->goTo(d.second);
		}
	}
	*/
}} // namespace grvc::mbzirc