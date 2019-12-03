#pragma once

#include <Eigen/Core>

namespace dsm
{
	// Gauss-Newton approximation of 
	// least squares Hessian: H*x=g

	template<int Row, int Col>
	class MatrixBlock
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	protected:

		// The hessian is stored as
		// double precision for precision

		Eigen::Matrix<double, Row, Col> H_;

	public:

		MatrixBlock();
		~MatrixBlock();

		// get matrix block
		const Eigen::Matrix<double, Row, Col>& block() const;

		// reset the block
		void reset();

		// add new measurement to the block
		void add(const Eigen::Matrix<float, 1, Row>& A,
				 const Eigen::Matrix<float, 1, Col>& B);

		// add new weighted measurement to the block
		void add(const Eigen::Matrix<float, 1, Row>& A,
				 const Eigen::Matrix<float, 1, Col>& B,
				 const float w);

		// fill the block lower triangular part symmetrically
		void fillSymmetric();
	};
}

// Generic Implementation
#include "MatrixBlock.hpp"

// Template Specializations
#include "MatrixBlock_8_8.hpp"
