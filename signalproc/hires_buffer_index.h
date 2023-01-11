
#pragma once


#include <stdint.h>



/*  Helper class for use as buffer indexes with fractional increase and operation.
 *  Internally, these are just uint32_t values for high speed. Maximum index therefore
 *  if 65535, and resolution is 1/65536.
 */
class HiresBufIx{
    public:

        HiresBufIx(double highres_index)
        : val(static_cast<uint32_t>(highres_index*FIXPT_FACTOR)) {}

        HiresBufIx(uint32_t int_index)
        : val(int_index*FIXPT_FACTOR) {}

        HiresBufIx(int int_index)
        : val(int_index*FIXPT_FACTOR) {}

        size_t index() {return val>>FIXPT_SHIFT;}

        double float_index() {return static_cast<double>(val)/FIXPT_FACTOR;}

        bool operator==(const HiresBufIx& rhs) {      
            return val==rhs.val;
        }

        bool operator==(uint32_t div) {      
            return index()==div;
        }

        bool operator>(const HiresBufIx& rhs) {      
            return val>rhs.val;
        }

        bool operator<(const HiresBufIx& rhs) {      
            return val<rhs.val;
        }

        bool operator>=(const HiresBufIx& rhs) {      
            return val>=rhs.val;
        }

        HiresBufIx& operator+=(const HiresBufIx& rhs) {      
            val+=rhs.val;                    
            return *this; // return the result by reference
        }

        friend HiresBufIx operator+(HiresBufIx lhs, const HiresBufIx& rhs) // otherwise, both parameters may be const references
        {
            lhs += rhs;
            return lhs; // return the result by value (uses move constructor)
        }

        HiresBufIx& operator-=(const HiresBufIx& rhs) {      
            val-=rhs.val;                    
            return *this; // return the result by reference
        }

        friend HiresBufIx operator-(HiresBufIx lhs, const HiresBufIx& rhs) // otherwise, both parameters may be const references
        {
            lhs -= rhs;
            return lhs; // return the result by value (uses move constructor)
        }


        HiresBufIx& operator/=(uint32_t div) {      
            val/=div;                    
            return *this; // return the result by reference
        }

        friend HiresBufIx operator/(HiresBufIx lhs, uint32_t div)
        {
            lhs /= div;
            return lhs; // return the result by value (uses move constructor)
        }


        HiresBufIx& operator*=(uint32_t mul) {      
            val*=mul;                    
            return *this; // return the result by reference
        }

        friend HiresBufIx operator*(HiresBufIx lhs, uint32_t mul)
        {
            lhs *= mul;
            return lhs; // return the result by value (uses move constructor)
        }


    private:
        static const uint32_t FIXPT_SHIFT = 16;
        static const uint32_t FIXPT_FACTOR = 1<<FIXPT_SHIFT;
        uint32_t val=0;  // internal representation as fixed point number
};



