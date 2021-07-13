/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

/*
 * Warning: please make sure you do NOT #include header files from
 * libraries other than Boost.Preprocessor. This header file provides a collection
 * of macros which can be used in any context as long as the types defined within
 * the macros are available at the point where they get expanded.
 *
 * So, please, please abstain to create extraneous dependencies!
 *
*/

#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/control/expr_if.hpp>
#include <boost/preprocessor/comparison/greater.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

#define LOT_OP_STRINGIZE(s, data, elem)               BOOST_PP_STRINGIZE(elem)
#define LOT_OP_BRACED_ENUM_VALUE_PAIR(s, data, elem)  BOOST_PP_COMMA_IF(BOOST_PP_GREATER(s,2)) {data::elem, valueOf<data::elem>()}

/*
 * Takes as input a sequence of n items and expands it to list of C-strings
 * separated by comma.
 *
 * Example:
 *  LOT_TO_STRINGIZE_ENUM((aa)(bb)(cc))  -> "aa", "bb", "cc"
*/
#define LOT_TO_STRINGIZE_ENUM(seq)  BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM(LOT_OP_STRINGIZE, 0, seq))

#define LOT_GENERATE_BRACED_ENUM_VALUE_PAIRS(seq, namespaceName) BOOST_PP_SEQ_FOR_EACH(LOT_OP_BRACED_ENUM_VALUE_PAIR, namespaceName, seq)
