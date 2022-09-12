// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "jntarray.hpp"

namespace KDL
{
    using namespace Eigen;

    JntArray::JntArray()
    {
    }

    JntArray::JntArray(std::size_t _size):
        data(_size)
    {
        data.setZero();
    }

    JntArray::JntArray(std::initializer_list<double> l)
    {
        data.resize(l.size());
        VectorXd::Index idx = 0;
        for(auto val : l)
            data[idx++] = val;
    }
    JntArray::JntArray(const std::vector<double>& v)
    {
        this->data = Eigen::VectorXd::Map(v.data(), v.size());
    }

    JntArray::JntArray(const JntArray& arg)
        : data(arg.data)
    {
    }

    JntArray::JntArray(JntArray &&arg) noexcept
        : data(std::move(arg.data))
    {
    }

    JntArray::JntArray(const VectorXd& arg)
        : data(arg)
    {
    }

    JntArray::JntArray(VectorXd&& arg) noexcept
        : data(std::move(arg))
    {
    }

    JntArray& JntArray::operator = (const JntArray& arg)
    {
        data=arg.data;
        return *this;
    }

    JntArray& JntArray::operator = (JntArray&& arg) noexcept
    {
        data = std::move(arg.data);
        return *this;
    }

    JntArray::~JntArray()
    {
    }

    void JntArray::resize(std::size_t newSize)
    {
        data.conservativeResizeLike(VectorXd::Zero(newSize));
    }

    double* JntArray::begin()
    {
        return data.data();
    }

    const double* JntArray::begin() const
    {
        return data.data();
    }

    double* JntArray::end()
    {
        return data.data()+data.size();
    }

    const double* JntArray::end() const
    {
        return data.data()+data.size();
    }

    std::vector<double> JntArray::toStdVector() const
    {
        std::vector<double> v(data.size());
        VectorXd::Map(v.data(), v.size()) = data;
        return v;
    }

    double JntArray::operator()(std::size_t i)const
    {
        return data(i);
    }

    double& JntArray::operator()(std::size_t i)
    {
        return data(i);
    }

    double JntArray::operator[](std::size_t i)const
    {
        return data(i);
    }

    double& JntArray::operator[](std::size_t i)
    {
        return data(i);
    }

    std::size_t JntArray::rows()const
    {
        return data.rows();
    }

    std::size_t JntArray::size()const
    {
        return data.size();
    }

    std::size_t JntArray::columns()const
    {
        return data.cols();
    }

    void Add(const JntArray& src1,const JntArray& src2,JntArray& dest)
    {
        dest.data=src1.data+src2.data;
    }

    void Subtract(const JntArray& src1,const JntArray& src2,JntArray& dest)
    {
        dest.data=src1.data-src2.data;
    }

    void Multiply(const JntArray& src,const double& factor,JntArray& dest)
    {
        dest.data=factor*src.data;
    }

    void Divide(const JntArray& src,const double& factor,JntArray& dest)
    {
        dest.data=src.data/factor;
    }

    void MultiplyJacobian(const Jacobian& jac, const JntArray& src, Twist& dest)
    {
        Eigen::Matrix<double,6,1> t=jac.data.lazyProduct(src.data);
        dest=Twist(Vector(t(0),t(1),t(2)),Vector(t(3),t(4),t(5)));
    }
    
    void SetToZero(JntArray& array)
    {
        array.data.setZero();
    }

    bool Equal(const JntArray& src1, const JntArray& src2,double eps)
    {
        if(src1.rows()!=src2.rows())
            return false;
        return (src1.data-src2.data).isZero(eps);
    }

    bool operator==(const JntArray& src1,const JntArray& src2){return Equal(src1,src2);}
    //bool operator!=(const JntArray& src1,const JntArray& src2){return Equal(src1,src2);}

}


