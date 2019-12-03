// Implementation of the generic VectorBlock

namespace dsm
{
	template<>
	inline void VectorBlock<8>::add(const Eigen::Matrix<float, 1, 8>& A,
									const float b)
	{
		// g = A.t() * b

		this->g_[0] += A[0] * b;
		this->g_[1] += A[1] * b;
		this->g_[2] += A[2] * b;
		this->g_[3] += A[3] * b;
		this->g_[4] += A[4] * b;
		this->g_[5] += A[5] * b;
		this->g_[6] += A[6] * b;
		this->g_[7] += A[7] * b;
	}

	template<>
	inline void VectorBlock<8>::add(const Eigen::Matrix<float, 1, 8>& A,
									const float b,
									const float w)
	{
		// g = A.t() * w * b

		const float wb = w * b;

		this->g_[0] += A[0] * wb;
		this->g_[1] += A[1] * wb;
		this->g_[2] += A[2] * wb;
		this->g_[3] += A[3] * wb;
		this->g_[4] += A[4] * wb;
		this->g_[5] += A[5] * wb;
		this->g_[6] += A[6] * wb;
		this->g_[7] += A[7] * wb;
	}
}