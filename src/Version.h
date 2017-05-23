/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifndef VERSION
# define VERSION "1.19alpha3"
#endif

#ifndef DATE
# define DATE "2017-05-09"
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman"

#ifdef POLYPRINTER
#undef VERSION
# define VERSION "1.18RC1Poly0003"
#endif

#endif /* SRC_VERSION_H_ */
