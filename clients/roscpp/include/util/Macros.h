/***************************************************************************************************
 * Copyright Five AI 2017.
 * All rights reserved.
 ***************************************************************************************************/
#pragma once

/*
 * Warning: please make sure you do NOT #include header files from
 * libraries other than Boost.Preprocessor. This header file provides a collection
 * of macros which can be used in any context as long as the types defined within
 * the macros are available at point where they get expanded.
 *
 * So, please, please abstain to create extraneous dependencies!
 *
*/

#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/control/expr_if.hpp>
#include <boost/preprocessor/comparison/greater.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <boost/preprocessor/seq/fold_left.hpp>
#include <boost/preprocessor/facilities/expand.hpp>
#include <boost/preprocessor/seq/pop_back.hpp>
#include <boost/preprocessor/seq/subseq.hpp>
#include <boost/preprocessor/tuple/elem.hpp>

#define FIVEAI_OP_STRINGIZE(s, data, elem)               BOOST_PP_STRINGIZE(elem)
#define FIVEAI_OP_AND_CHAINED_CALLS(s, data, elem)       BOOST_PP_EXPR_IF(BOOST_PP_GREATER(s,2), &&) data(elem)
#define FIVEAI_OP_COLLECT_WRITER_CALL(s, data, elem)     collectWriterIfEnabled<data::elem>() ;
#define FIVEAI_OP_DISABLE_WRITER_CALL(s, data, elem)     getParams<data::elem>().enabled = false;
#define FIVEAI_OP_DOUBLE_COLON_FOLD(s, state, x)         state::x
#define FIVEAI_OP_BEGIN_NAMESPACE(s, data, elem)         namespace elem {
#define FIVEAI_OP_END_NAMESPACE(s, data, elem)           }
#define FIVEAI_OP_BRACED_ENUM_VALUE_PAIR(s, data, elem)  BOOST_PP_COMMA_IF(BOOST_PP_GREATER(s,2)) {data::elem, valueOf<data::elem>()}

/*
 * Takes as input a sequence of n items and expands it to list of C-strings
 * separated by comma.
 *
 * Example:
 *  FIVEAI_TO_STRINGIZE_ENUM((aa)(bb)(cc))  -> "aa", "bb", "cc"
*/
#define FIVEAI_TO_STRINGIZE_ENUM(seq)                    BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM(FIVEAI_OP_STRINGIZE, 0, seq))

/*
 * Takes as input a sequence of n items and a function call.
 *
 * Example:
 *  FIVEAI_AND_CHAINED_CALLS((aa)(bb)(cc), function)  -> function(aa) && function(bb) && function(cc)
*/
#define FIVEAI_AND_CHAINED_CALLS(seq, call)              BOOST_PP_SEQ_FOR_EACH(FIVEAI_OP_AND_CHAINED_CALLS, call, seq)

/*
 * Concatenate the elements of the @param seq into '::' separated string
 * Example:
 *  FIVEAI_DOUBLE_COLON_FOLD((a1)(a2)(a3))  ->  a1::a2::a3
*/
#define FIVEAI_DOUBLE_COLON_FOLD(seq)  BOOST_PP_SEQ_FOLD_LEFT(FIVEAI_OP_DOUBLE_COLON_FOLD, BOOST_PP_EMPTY(), seq)

/*
 * Takes as input a sequence of n items. The first (n-1) elements represent
 * the namespaces within which the class identified by the last element would
 * live in.
 *
 * The sequence gets expanded into the declaration of the namespaces identified
 * by the first n-1 items.
 *
 * Example:
 *  FIVEAI_BEGIN_DECLARE_NAMESPACE((n1)(n2)(n3)(c1))  ->  namespace n1 { namespace n2 { namespace n3 {
 *  FIVEAI_END_DECLARE_NAMESPACE((n1)(n2)(n3)(c1))     -> }}}
 *
 *  FIVEAI_BEGIN_DECLARE_NAMESPACE((c1))  ->  <empty>
 *  FIVEAI_END_DECLARE_NAMESPACE((c1))     -> <empty>
*/
#define FIVEAI_BEGIN_DECLARE_NAMESPACE(seq)  BOOST_PP_EXPR_IF(BOOST_PP_SEQ_SIZE(BOOST_PP_SEQ_POP_BACK(seq)), BOOST_PP_SEQ_FOR_EACH(FIVEAI_OP_BEGIN_NAMESPACE, ~, BOOST_PP_SEQ_POP_BACK(seq)) )
#define FIVEAI_END_DECLARE_NAMESPACE(seq)    BOOST_PP_EXPR_IF(BOOST_PP_SEQ_SIZE(BOOST_PP_SEQ_POP_BACK(seq)), BOOST_PP_SEQ_FOR_EACH(FIVEAI_OP_END_NAMESPACE, ~, BOOST_PP_SEQ_POP_BACK(seq)) )

/*
 * Takes as input a sequence of n items. The first (n-1) elements represent
 * the namespaces within which the class identified by the last element would
 * live in.
 *
 * The sequence gets expanded into the declaration of the class identified
 * by the last item.
 *
 * Example:
 *  FIVEAI_DECLARE_CLASS((n1)(n2)(n3)(c1))  ->  class c1;
 *  FIVEAI_DECLARE_CLASS((c1))  ->  class c1;
*/
#define FIVEAI_DECLARE_CLASS(seq)  class BOOST_PP_SEQ_ELEM(BOOST_PP_SUB(BOOST_PP_SEQ_SIZE(seq),1), seq) ;

/*
 * Takes as input a sequence of n items. The first (n-1) elements represent
 * the namespaces within which the class identified by the last element would
 * live in.
 *
 * The sequence gets expanded into the declaration of the class identified
 * by the last item within the namespaces identified by the first (n-1) items.
 *
 * Example:
 *  FIVEAI_DECLARE_CLASS_IN_NAMESPACE((n1)(n2)(n3)(c1))  -> namespace n1 { namespace n2 { namespace n3 {  class c1; }}}
 *  FIVEAI_DECLARE_CLASS_IN_NAMESPACE((c1))              -> class c1;
*/
#define FIVEAI_DECLARE_CLASS_IN_NAMESPACE(seq) \
    FIVEAI_BEGIN_DECLARE_NAMESPACE(seq)        \
        FIVEAI_DECLARE_CLASS(seq)              \
    FIVEAI_END_DECLARE_NAMESPACE(seq)

#define FIVEAI_GENERATE_BRACED_ENUM_VALUE_PAIRS(seq, namespaceName) BOOST_PP_SEQ_FOR_EACH(FIVEAI_OP_BRACED_ENUM_VALUE_PAIR, namespaceName, seq)
#define FIVEAI_GENERATE_COLLECT_WRITER_CALLS(seq, namespaceName)    BOOST_PP_SEQ_FOR_EACH(FIVEAI_OP_COLLECT_WRITER_CALL, namespaceName, seq)
#define FIVEAI_GENERATE_DISABLE_WRITER_CALLS(seq, namespaceName)    BOOST_PP_SEQ_FOR_EACH(FIVEAI_OP_DISABLE_WRITER_CALL, namespaceName, seq)
