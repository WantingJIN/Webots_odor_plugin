// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classRandomSelector
#define classRandomSelector

template <typename ElementType, unsigned cCacheSize>
class RandomSelector;

#include "Random.h"

#define THISTEMPLATE template <typename ElementType, unsigned cCacheSize>

//! Randomly selects an item of an iterated list.
/*!
	This class is useful to select an item randomly from a list that is not know a priori.

	The classical method to solve this problem is to iterate twice over the same list, once
	to count the number of elements and once to iterate to the randomly selected element.
	This method is in some cases expensive (when iterating is expensive) and in some cases
	even impossible (e.g. for non-deterministic lists).

	This class uses another method which allows to select an item during the first iteration.
*/
template <typename ElementType, unsigned cCacheSize>
class RandomSelector {

protected:
	ElementType mCache[cCacheSize];
	int mCacheCount;
	ElementType mHistoryRandom;
	int mHistoryCount;
	ElementType mDefault;

	//! Performs the conjoin operation which adds the cache to the history.
	void Conjoin();

public:
	//! Constructor.
	RandomSelector(): mCacheCount(0), mHistoryRandom(), mHistoryCount(0), mDefault() {}
	//! Destructor
	~RandomSelector() {}

	//! Sets the default element. This is returned if no element is added.
	void SetDefault(ElementType element) {
		mDefault = element;
	}
	//! Restarts at the beginning.
	void Start();
	//! Adds an element to set.
	void Add(ElementType element);
	//! Returns a random element from the set.
	ElementType GetRandom();
};

#define THISCLASS RandomSelector<ElementType, cCacheSize>

THISTEMPLATE
void THISCLASS::Start() {
	mCacheCount = 0;
	mHistoryCount = 0;
}

THISTEMPLATE
void THISCLASS::Conjoin() {
	if (mCacheCount == 0) {
		return;
	}

	Random rand;
	double choosehist = ((double)(mHistoryCount)) / ((double)(mHistoryCount + mCacheCount));
	if (rand.Binary(choosehist) == 0) {
		int randid = rand.Uniform(0, mCacheCount);
		mHistoryRandom = mCache[randid];
	}

	mHistoryCount += mCacheCount;
	mCacheCount = 0;
}

THISTEMPLATE
void THISCLASS::Add(ElementType element) {
	if (mCacheCount >= cCacheSize) {
		Conjoin();
	}

	mCache[mCacheCount] = element;
	mCacheCount++;
}

THISTEMPLATE
ElementType THISCLASS::GetRandom() {
	Conjoin();
	return (mHistoryCount ? mHistoryRandom : mDefault);
}

#undef THISTEMPLATE
#undef THISCLASS

#endif
