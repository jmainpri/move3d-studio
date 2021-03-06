/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Tue Mar 17 18:04:54 2009
 */
#ifndef __CEXTRACT__

extern void printGridVals ( void );
extern void printObstacles ( void );
extern void printBitmap ( void );
extern void printCombinedBitmap ( void );
extern void printGridObst ( void );
extern int igetCellCoord ( float cLimInf, float cLimSup, int maxGridVal, float cRealCoord );
extern int InitWaveCells ( float x1, float y1, float x2, float y2, float waveX, float waveY, hri_bitmapset* PSP_BTSET );
extern int iget_wave_cost ( float x, float y );
extern long getMaxGridCost ( void );
extern int addWave(float x1, float y1, float x2, float y2, float waveX, float waveY);
extern void putGrid(int maxX, int maxY);
extern void pushGrid(int maxX, int maxY);
extern int iget_all_wave_cost(float x, float y);
extern long getMaxWaveCost();

#endif /* __CEXTRACT__ */
