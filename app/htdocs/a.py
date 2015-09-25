import subprocess

plot = subprocess.Popen(['gnuplot'], stdin=subprocess.PIPE)
plot.stdin.write( "set term gif size 400,200 font dejavu;" )
plot.stdin.write( 'set output "gnuplot.gif";' )
plot.stdin.write( "plot x;" )

#plot.stdin.write( "1" )
#plot.stdin.write( " " )
#plot.stdin.write( "2" )
#plot.stdin.write( "\\n" )
#plot.stdin.write( "EOF\\n" )
plot.stdin.write( "quit\n" )
