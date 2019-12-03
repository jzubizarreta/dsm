#pragma once

#include <Eigen/Core>

namespace dsm
{
	// Least squares vector block

	template<int Row>
	class VectorBlock
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	protected:

		// The hessian is stored as
		// double precision to maintain the
		// robustness to large summations

		Eigen::Matrix<double, Row, 1> g_;

	public:

		VectorBlock();
		~VectorBlock();

		// get vector block
		const Eigen::Matrix<double, Row, 1>& block() const;

		// reset the block
		void reset();

		// add new measurement to the block
		void add(const Eigen::Matrix<float, 1, Row>& A,
				 const float b);

		// add new weighted measurement to the block
		void add(const Eigen::Matrix<float, 1, Row>& A,
				 const float b,
				 const float w);
	};
}

// Generic Implementation
#include "VectorBlock.hpp"

// Template Specializations
#include "VectorBlock_8.hpp"