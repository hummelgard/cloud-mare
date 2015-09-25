import subprocess

plot = subprocess.Popen(['gnuplot'], shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
plot.stdin.write( bytes("set term gif size 400,200 font dejavu\n","UTF-8") )
plot.stdin.write( bytes('set output "/srv/http/app/htdocs/static/gnuplot.gif"\n',"UTF-8") )
plot.stdin.write( bytes("plot '-' using 1:2\n","UTF-8") )


for i in range(10):
  plot.stdin.write( bytes("1", "UTF-8") )
  plot.stdin.write( bytes(" ", "UTF-8") )
  plot.stdin.write( bytes("2\n", "UTF-8") )
  plot.stdin.write( bytes("2", "UTF-8") )
  plot.stdin.write( bytes(" ", "UTF-8") )
  plot.stdin.write( bytes("4\n", "UTF-8") )

plot.stdin.write( bytes("\n", "UTF-8") )
plot.stdin.write( bytes("EOF\n","UTF-8") )
plot.stdin.write( bytes("quit\n","UTF-8") )
plot.stdin.flush()
plot.wait()
