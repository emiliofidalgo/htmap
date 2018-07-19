Using the Caltech 101 splits provided on the web and an SVM Classifier you should have the following results:

		Shape 180	Shape180	Shape360	Shape360
		(whole image)	(roi)		(whole image)	(roi)
l=0		13.20		14.70		16.20		17.57
l=1		33.07		38.56		36.14		44.37
l=2		46.99		58.76		47.25		61.69
l=3		48.43		61.04		49.28		61.63

merge 
(exhaustive 
search)		51.76		64.31		54.33		66.27

merge
(varma 
method)		49.21		60.19		50.00		61.04

NOTE: SVM 1-vs-all has been used to learn the level weights when merging all the pyramid levels.

------------------------------------------------

Using the Caltech 256 splits provided on the web and an SVM Classifier you should have the following results:

		Shape 180	Shape180	Shape360	Shape360
		(whole image)	(roi)		(whole image)	(roi)

merge 		
(exhaustive 
search)		16.81		24.28		19.31		27.17



NOTE: again, SVM 1-vs-all has been used to learn the level weights when merging all the pyramid levels.