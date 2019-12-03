// Implementation of the generic HessianBlock

namespace dsm
{
	template<int Row, int Col>
	inline MatrixBlock<Row, Col>::MatrixBlock()
	{
		this->reset();
	}

	template<int Row, int Col>
	inline MatrixBlock<Row, Col>::~MatrixBlock()
	{
	}

	template<int Row, int Col>
	inline const Eigen::Matrix<double, Row, Col>& MatrixBlock<Row, Col>::block() const
	{
		return this->H_;
	}

	template<int Row, int Col>
	inline void MatrixBlock<Row, Col>::reset()
	{
		this->H_.setZero();
	}

	template<int Row, int Col>
	inline void MatrixBlock<Row, Col>::add(const Eigen::Matrix<float, 1, Row>& A,
										   const Eigen::Matrix<float, 1, Col>& B)
	{
		// fill block upper triangular part
		for (int r = 0; r < Row; ++r)
		{
			for (int c = r; c < Col; ++c)
			{
				// H = A.t() * B
				this->H_(r, c) += A[r] * B[c];
			}
		}

		//this->H_.noalias() += (A.transpose() * B).template cast<double>();
	}

	template<int Row, int Col>
	inline void MatrixBlock<Row, Col>::add(const Eigen::Matrix<float, 1, Row>& A,
										   const Eigen::Matrix<float, 1, Col>& B,
										   const float w)
	{
		// fill block upper triangular part
		for (int r = 0; r < Row; ++r)
		{
			const float Aw = A[r] * w;

			for (int c = r; c < Col; ++c)
			{
				// H = A.t() * w * B
				this->H_(r, c) += Aw * B[c];
			}
		}

		//this->H_.noalias() += (A.transpose() * w * B).template cast<double>();
	}

	template<int Row, int Col>
	inline void MatrixBlock<Row, Col>::fillSymmetric()
	{
		// fill block lower triangular part
		for (int r = 0; r < Row; ++r)
		{
			for (int c = r + 1; c < Col; ++c)
			{
				this->H_(c, r) = this->H_(r, c);
			}
		}
	}
}