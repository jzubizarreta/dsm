// Implementation of the specialization MatrixBlock<8, 8>

namespace dsm
{
	template<>
	inline void MatrixBlock<8, 8>::add(const Eigen::Matrix<float, 1, 8>& A,
									   const Eigen::Matrix<float, 1, 8>& B)
	{
		const float& A0 = A[0];
		const float& A1 = A[1];
		const float& A2 = A[2];
		const float& A3 = A[3];
		const float& A4 = A[4];
		const float& A5 = A[5];
		const float& A6 = A[6];
		const float& A7 = A[7];

		const float& B0 = B[0];
		const float& B1 = B[1];
		const float& B2 = B[2];
		const float& B3 = B[3];
		const float& B4 = B[4];
		const float& B5 = B[5];
		const float& B6 = B[6];
		const float& B7 = B[7];

		// fill block upper triangular part
		// H = A.t() * B

		this->H_(0, 0) += A0 * B0;
		this->H_(0, 1) += A0 * B1;
		this->H_(0, 2) += A0 * B2;
		this->H_(0, 3) += A0 * B3;
		this->H_(0, 4) += A0 * B4;
		this->H_(0, 5) += A0 * B5;
		this->H_(0, 6) += A0 * B6;
		this->H_(0, 7) += A0 * B7;
						  	   
		this->H_(1, 1) += A1 * B1;
		this->H_(1, 2) += A1 * B2;
		this->H_(1, 3) += A1 * B3;
		this->H_(1, 4) += A1 * B4;
		this->H_(1, 5) += A1 * B5;
		this->H_(1, 6) += A1 * B6;
		this->H_(1, 7) += A1 * B7;
						  	   
		this->H_(2, 2) += A2 * B2;
		this->H_(2, 3) += A2 * B3;
		this->H_(2, 4) += A2 * B4;
		this->H_(2, 5) += A2 * B5;
		this->H_(2, 6) += A2 * B6;
		this->H_(2, 7) += A2 * B7;
						  	   
		this->H_(3, 3) += A3 * B3;
		this->H_(3, 4) += A3 * B4;
		this->H_(3, 5) += A3 * B5;
		this->H_(3, 6) += A3 * B6;
		this->H_(3, 7) += A3 * B7;
						  	   
		this->H_(4, 4) += A4 * B4;
		this->H_(4, 5) += A4 * B5;
		this->H_(4, 6) += A4 * B6;
		this->H_(4, 7) += A4 * B7;
						  	   
		this->H_(5, 5) += A5 * B5;
		this->H_(5, 6) += A5 * B6;
		this->H_(5, 7) += A5 * B7;
						  	   
		this->H_(6, 6) += A6 * B6;
		this->H_(6, 7) += A6 * B7;
						 	   
		this->H_(7, 7) += A7 * B7;
	}

	template<>
	inline void MatrixBlock<8, 8>::add(const Eigen::Matrix<float, 1, 8>& A,
									   const Eigen::Matrix<float, 1, 8>& B,
									   const float w)
	{
		const float& A0 = A[0];
		const float& A1 = A[1];
		const float& A2 = A[2];
		const float& A3 = A[3];
		const float& A4 = A[4];
		const float& A5 = A[5];
		const float& A6 = A[6];
		const float& A7 = A[7];

		const float& B0 = B[0];
		const float& B1 = B[1];
		const float& B2 = B[2];
		const float& B3 = B[3];
		const float& B4 = B[4];
		const float& B5 = B[5];
		const float& B6 = B[6];
		const float& B7 = B[7];

		const float Aw0 = A0 * w;
		const float Aw1 = A1 * w;
		const float Aw2 = A2 * w;
		const float Aw3 = A3 * w;
		const float Aw4 = A4 * w;
		const float Aw5 = A5 * w;
		const float Aw6 = A6 * w;
		const float Aw7 = A7 * w;

		// fill block upper triangular part
		// H = A.t() * w * B

		this->H_(0, 0) += (double)(Aw0 * B0);
		this->H_(0, 1) += Aw0 * B1;
		this->H_(0, 2) += Aw0 * B2;
		this->H_(0, 3) += Aw0 * B3;
		this->H_(0, 4) += Aw0 * B4;
		this->H_(0, 5) += Aw0 * B5;
		this->H_(0, 6) += Aw0 * B6;
		this->H_(0, 7) += Aw0 * B7;
						  		
		this->H_(1, 1) += Aw1 * B1;
		this->H_(1, 2) += Aw1 * B2;
		this->H_(1, 3) += Aw1 * B3;
		this->H_(1, 4) += Aw1 * B4;
		this->H_(1, 5) += Aw1 * B5;
		this->H_(1, 6) += Aw1 * B6;
		this->H_(1, 7) += Aw1 * B7;
						  		
		this->H_(2, 2) += Aw2 * B2;
		this->H_(2, 3) += Aw2 * B3;
		this->H_(2, 4) += Aw2 * B4;
		this->H_(2, 5) += Aw2 * B5;
		this->H_(2, 6) += Aw2 * B6;
		this->H_(2, 7) += Aw2 * B7;
						  		
		this->H_(3, 3) += Aw3 * B3;
		this->H_(3, 4) += Aw3 * B4;
		this->H_(3, 5) += Aw3 * B5;
		this->H_(3, 6) += Aw3 * B6;
		this->H_(3, 7) += Aw3 * B7;
						  		
		this->H_(4, 4) += Aw4 * B4;
		this->H_(4, 5) += Aw4 * B5;
		this->H_(4, 6) += Aw4 * B6;
		this->H_(4, 7) += Aw4 * B7;
						  		
		this->H_(5, 5) += Aw5 * B5;
		this->H_(5, 6) += Aw5 * B6;
		this->H_(5, 7) += Aw5 * B7;
						  		
		this->H_(6, 6) += Aw6 * B6;
		this->H_(6, 7) += Aw6 * B7;
						  		
		this->H_(7, 7) += Aw7 * B7;
	}

	template<>
	inline void MatrixBlock<8, 8>::fillSymmetric()
	{
		// fill block lower triangular part

		this->H_(1, 0) = this->H_(0, 1);

		this->H_(2, 0) = this->H_(0, 2);
		this->H_(2, 1) = this->H_(1, 2);

		this->H_(3, 0) = this->H_(0, 3);
		this->H_(3, 1) = this->H_(1, 3);
		this->H_(3, 2) = this->H_(2, 3);

		this->H_(4, 0) = this->H_(0, 4);
		this->H_(4, 1) = this->H_(1, 4);
		this->H_(4, 2) = this->H_(2, 4);
		this->H_(4, 3) = this->H_(3, 4);

		this->H_(5, 0) = this->H_(0, 5);
		this->H_(5, 1) = this->H_(1, 5);
		this->H_(5, 2) = this->H_(2, 5);
		this->H_(5, 3) = this->H_(3, 5);
		this->H_(5, 4) = this->H_(4, 5);

		this->H_(6, 0) = this->H_(0, 6);
		this->H_(6, 1) = this->H_(1, 6);
		this->H_(6, 2) = this->H_(2, 6);
		this->H_(6, 3) = this->H_(3, 6);
		this->H_(6, 4) = this->H_(4, 6);
		this->H_(6, 5) = this->H_(5, 6);

		this->H_(7, 0) = this->H_(0, 7);
		this->H_(7, 1) = this->H_(1, 7);
		this->H_(7, 2) = this->H_(2, 7);
		this->H_(7, 3) = this->H_(3, 7);
		this->H_(7, 4) = this->H_(4, 7);
		this->H_(7, 5) = this->H_(5, 7);
		this->H_(7, 6) = this->H_(6, 7);
	}
}