it seems that updating the vectors of lines fter trimming can prevent valid solutions in the future.

I believe I need to maintain two solution paths:
1. One when there is a global look ahead
2. One when there is only a local look ahead
The junction solver should remain pure and not have any specifics for lines or arcs. 
The checks for inverted line direction are probably only useful when we're not doing a global look ahead. 
The global look ahead can give us better results, staying closer to the original profile. 
I am chasing my tail trying to accommodate both situations. 

The pausing and restoring of Cutter-Comp needs work. 
I believe I need to separate out the comp-in from the steady work 
and allow offsetting to happen without the comp-in and comp-out logic. 