//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef sbpl_Discretizer_h
#define sbpl_Discretizer_h

namespace sbpl {

/// \brief Discretizes a continuous space into uniform cells such that one cell
///     is centered on a given minimum value.
class MinDiscretizer
{
public:

    MinDiscretizer(double res, double min) : m_res(res), m_min(min) { }

    double res() const { return m_res; }
    double min() const { return m_min; }

    int discretize(double d) const {
        return 0;
    }

    double continuize(int i) const {
        return 0;
    }

private:

    double m_res;
    double m_min;
};

/// \brief Discretizes a continuous space into uniform cells such that one cell
///     is centered on zero.
class ZeroDiscretizer
{
public:

    ZeroDiscretizer(double res) : m_res(res) { }

    double res() const { return m_res; }

    int discretize(double d) const {
        return 0;
    }

    double continuize(int i) const {
        return 0;
    }

private:

    double m_res;
};

/// \brief Discretizes a continuous space into uniform cells such that one cell
///     is centered on half res (so a cell boundary exists at 0)
class HalfResDiscretizer
{
public:

    HalfResDiscretizer(double res) : m_res(res) { }

    double res() const { return m_res; }

    int discretize(double d) const {
        return (d >= 0) ? (int)(d / m_res) : ((int)(d / m_res) - 1);
    }

    double continuize(int i) const {
        return (double)i * m_res + 0.5 * m_res;
    }

private:

    double m_res;
};

} // namespace sbpl

#endif
