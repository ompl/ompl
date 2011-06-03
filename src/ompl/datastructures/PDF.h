#ifndef OMPL_DATASTRUCTURES_PDF_
#define OMPL_DATASTRUCTURES_PDF_

#include <vector>

namespace ompl
{
	template <typename _T>
	class PDF
	{
		public:

		PDF(void) : normalization(0.0);
		{
		}

		virtual ~PDF(void)
		{
		}

		virtual void add(_T d, const double w)
		{
			data.push_back(d);
			weights.push_back(w);
			normalization += w;
		}

		virtual _T sample(double r) const
		{
			r *= normalization;
			//TODO
		}

		protected:
		std::vector<double> weights;
		double normalization;
		std::vector<_T> data;
	};
}

#endif
