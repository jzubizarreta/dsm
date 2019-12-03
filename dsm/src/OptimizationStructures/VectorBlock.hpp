// Implementation of the generic VectorBlock

namespace dsm
{
	template<int Row>
	inline VectorBlock<Row>::VectorBlock()
	{
		this->reset();
	}

	template<int Row>
	inline VectorBlock<Row>::~VectorBlock()
	{
	}

	template<int Row>
	inline const Eigen::Matrix<double, Row, 1>& VectorBlock<Row>::block() const
	{
		return this->g_;
	}

	template<int Row>
	inline void VectorBlock<Row>::reset()
	{
		this->g_.setZero();
	}

	template<int Row>
	inline void VectorBlock<Row>::add(const Eigen::Matrix<float, 1, Row>& A,
									  const float b)
	{
		// g = A.t() * b
		this->g_.noalias() += (A.transpose() * b).template cast<double>();
	}

	template<int Row>
	inline void VectorBlock<Row>::add(const Eigen::Matrix<float, 1, Row>& A,
									  const float b,
									  const float w)
	{
		// g = A.t() * w * b
		this->g_.noalias() += (A.transpose() * w * b).template cast<double>();
	}
}