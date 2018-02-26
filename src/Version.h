/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifndef VERSION
# define VERSION "1.21RC2"
#endif

#ifndef DATE
# define DATE "2018-02-15 build 2"
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman"

#ifdef POLYPRINTER
// would like to simply extdn it...
//#define PPCAT( A, B ) A ## B
#define VERSION_TEMP( s ) VERSION
#undef VERSION
# define VERSION "1.21RC2" "Poly0004"
#endif

#endif /* SRC_VERSION_H_ */
