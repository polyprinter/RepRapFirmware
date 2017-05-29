/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifndef VERSION
# define VERSION "1.19beta1"
#endif

#ifndef DATE
# define DATE "2017-05-29"
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman"

#ifdef POLYPRINTER
#undef VERSION
# define VERSION "1.19beta1Poly0004"
#endif

#endif /* SRC_VERSION_H_ */
