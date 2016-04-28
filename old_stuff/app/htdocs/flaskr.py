# all the imports
import sqlite3, math, subprocess
from flask import Flask, request, session, g, redirect, url_for, \
     abort, render_template, flash

# configuration
DATABASE = '/srv/http/app/flaskr.db'
DEBUG = True
SECRET_KEY = 'development key'
USERNAME = 'admin'
PASSWORD = 'default'

# create our little application :)
app = Flask(__name__)
app.config.from_object(__name__)
#app.config.from_envvar('FLASKR_SETTINGS', silent=True)

def connect_db():
    return sqlite3.connect(app.config['DATABASE'])


from contextlib import closing
def init_db():
    with closing(connect_db()) as db:
        with app.open_resource('schema.sql', mode='r') as f:
            db.cursor().executescript(f.read())
        db.commit()

@app.before_request
def before_request():
    g.db = connect_db()

@app.teardown_request
def teardown_request(exception):
    db = getattr(g, 'db', None)
    if db is not None:
        db.close()

@app.route('/')
def show_entries():
    cur = g.db.execute('select title, text from entries order by id desc')
    entries = [dict(title=row[0], text=row[1]) for row in cur.fetchall()]
    return render_template('show_entries.html', entries=entries)

@app.route('/battplot')
def show_battplot():
    cur = g.db.execute('select date, time, value0, value1 from positions order by id asc' )
     
    data = cur.fetchall()
    logStart = data[0]
    plotStart = ( str(logStart[0][0:2]) + "/" + str(logStart[0][3:5]) + "/" +str(logStart[0][6:8]) +
                   "-" + str(logStart[1][0:2]) + ":" + str(logStart[1][3:5]) + ":" +str(logStart[1][6:8]) )

    logEnd = data[-1]
    plotEnd = ( str(logEnd[0][0:2]) + "/" + str(logEnd[0][3:5]) + "/" +str(logEnd[0][6:8]) +
                   "-" + str(logEnd[1][0:2]) + ":" + str(logEnd[1][3:5]) + ":" +str(logEnd[1][6:8]) )
    print(plotStart)
    print(plotEnd)
    plot = subprocess.Popen(['gnuplot'], stdin=subprocess.PIPE)
    plot.stdin.write( bytes("set term gif size 600,300 font \"dejavu,8\" background '#FFFFFCF6';","UTF-8") )
    plot.stdin.write( bytes("set format y \"%3.0f\";","UTF-8") )
    plot.stdin.write( bytes("set ylabel 'Battery Charge (%)';","UTF-8") )

    plot.stdin.write( bytes("set grid;","UTF-8") )
    plot.stdin.write( bytes("set ytics 10;","UTF-8") )

    plot.stdin.write( bytes("set timefmt \"%d/%m/%y-%H:%M:%S\";","UTF-8") )
    plot.stdin.write( bytes("set xdata time;","UTF-8") )
    plot.stdin.write( bytes("set xtics format \"%d/%m %H:%M\" time rotate by -45;","UTF-8") )
    plot.stdin.write( bytes("set xtics \"","UTF-8") )
    plot.stdin.write( bytes(plotStart, "UTF-8") )
    plot.stdin.write( bytes("\", 86400, \"", "UTF-8") )
    plot.stdin.write( bytes(plotEnd, "UTF-8") )
    plot.stdin.write( bytes("\";","UTF-8") )

    plot.stdin.write( bytes('set output "/srv/http/app/htdocs/static/gnuplot.gif";',"UTF-8") )
    plot.stdin.write( bytes("plot [*:*][0:100] '-' using 1:2 with lines title '' lw 3 lc rgb '#B4A9D4'\n","UTF-8") )

    points = []
    for row in data:

      timemark = ( str(row[0][0:2]) + "/" + str(row[0][3:5]) + "/" +str(row[0][6:8]) +
                   "-" + str(row[1][0:2]) + ":" + str(row[1][3:5]) + ":" +str(row[1][6:8]) )
      
      points.append( dict(timeStamp=timemark, battPercent=row[2], battVoltage=row[3]) )

      plot.stdin.write( bytes(timemark, "UTF-8") )
      plot.stdin.write( bytes(" ", "UTF-8") )
      plot.stdin.write( bytes(row[2], "UTF-8") )
      plot.stdin.write( bytes("\n", "UTF-8") )

    plot.stdin.write( bytes("EOF\n","UTF-8") )
    plot.stdin.write( bytes("quit\n","UTF-8") )
    plot.stdin.flush()
    plot.wait()
    return render_template('show_battplot.html',points=points)


@app.route('/tempmap')
def show_tempmap():
    cur = g.db.execute('select latitude, longitude, value4 from positions ' )
    posData=[] 
    for row in cur.fetchall():
      posData.append(row)
    
    dataTransp = list(map(list, zip(*posData)))
    
    #convert strings to float, int
    latitude = list(map(float, dataTransp[0]))
    longitude = list(map(float, dataTransp[1]))	
    value2 = list(map(float, dataTransp[2]))

    #latitude width
    width = int( (max(latitude)-min(latitude) )*10000)
      
    #longitude width
    height = int( (max(longitude)-min(longitude) )*10000)
    
    #ortsjon
    latCenter = 6216576
    lonCenter = 1717866

    density = 3
   
    points=[]
    latScaleFactor=math.ceil(1.0/math.sin(math.radians( max(latitude)) ))
    for y in range(int(min(latitude)*100000)-1, int(max(latitude)*100000)+1, density):
      for x in range(int(min(longitude)*100000)-1, int(max(longitude)*100000)+1, density*latScaleFactor):    
    #for y in range(latCenter-50,latCenter+50,5):
    #  for x in range(lonCenter-150,lonCenter+50,5*latScaleFactor):
       
        avg_count=1
        temp=0
        for i in range(len(posData)):
         
          x0 = int(float(posData[i][1])*100000)
          y0 = int(float(posData[i][0])*100000)
           
          if( ( abs(x-x0) + abs(y-y0)*latScaleFactor ) < 3*latScaleFactor ):
            temp = temp + (value2[i])
            avg_count = avg_count + 1
        temp = temp / avg_count
        #if(temp != 0):
        points.append( dict(latitude=str(y/100000.0),longitude=str(x/100000.0),value2=str(temp) ) )

    return render_template('show_tempmap.html',points=points)


@app.route('/heatmap')
def show_heatmap():
    cur = g.db.execute('select latitude, longitude from positions ' )
    points = [dict(latitude=row[0], longitude=row[1]) 
              for row in cur.fetchall()]
    return render_template('show_heatmap.html',points=points)

@app.route('/posdata')
def show_posdata_entries():
    cur = g.db.execute('select latitude, longitude, date, time, value5, '+
                       'value5, value7 from positions order by id desc')
    positions = [dict(latitude=row[0], longitude=row[1], date=row[2], 
                      time=row[3], acx=row[4], acy=row[5], acz=row[6]) 
                      for row in cur.fetchall()]
    
    #positions = [dict(IMEI_id=row[0], latitude=row[1], longitude=row[2], 
    #             date=row[3], time=row[4], value1=row[5], 
    #             value2=row[6], value3=row[7]) for row in cur.fetchall()]
    return render_template('show_posdata_entries.html', positions=positions)


@app.route('/gps')
def show_gps_entries():
    cur = g.db.execute('select imei, imsi, version, name, latitude, longitude, date, time,' +
                       ' value0, value1, value2, value3, value4,' + 
                       ' value5, value6, value7, value8, value9, value10' + 
                       ' from positions order by id desc')
    positions = [dict(IMEI=row[0], IMSI=row[1], version=row[2], name=row[3], latitude=row[4],
                 longitude=row[5], 
                 date=row[6], time=row[7], value0=row[8], value1=row[9], 
                 value2=row[10], value3=row[11], value4=row[12], value5=row[13], 
                 value6=row[14], value7=row[15], value8=row[16], value9=row[17], 
                 value10=row[18]) for row in cur.fetchall()]
    
    #positions = [dict(IMEI_id=row[0], latitude=row[1], longitude=row[2], 
    #             date=row[3], time=row[4], value1=row[5], 
    #             value2=row[6], value3=row[7]) for row in cur.fetchall()]
    return render_template('show_gps_entries.html', positions=positions)


@app.route('/addData', methods=['POST'])
def addData_entry():
    #if not session.get('logged_in'):
    #    abort(401)
    version = request.form['ver']
    IMEI = request.form['IMEI']
    IMSI = request.form['IMSI']
    name = request.form['name']
    data = request.form['data']
    sum = int(request.form['sum'])
    check_string =("ver=" + version + "&IMEI=" + IMEI + "&IMSI=" + IMSI + "&name=" 
                   + name + "&data=" + data + "&sum=")
    check_sum = ''.join(format(ord(x), 'b') for x in check_string).count('1')
    data_array = data.split('#')
    length = len(data_array)    
    print(data_array)   
    if check_sum == sum:
       
        for i in range(0,length,15):
            #if version == 1:

                value0 = data_array[i] #batt %
                value1 = data_array[i+1] #batt milliVolt
                value2 = data_array[i+2] #DHT11 hum
                value3 = data_array[i+3] #DHT11 temp
                value4 = data_array[i+4] #MPU temp
                value5 = data_array[i+5] #acx
                value6 = data_array[i+6] #acy
                value7 = data_array[i+7] #acz
                value8 = data_array[i+8] #max
                value9 = data_array[i+9] #may
                value10 = data_array[i+10] #maz

                lat = data_array[i+11]
                lon = data_array[i+12]

                time = (data_array[i+13][0] + data_array[i+13][1] + ":" +
                        data_array[i+13][2] + data_array[i+13][3] + ":" +
                        data_array[i+13][4] + data_array[i+13][5])

                date = (data_array[i+14][0] + data_array[i+14][1] + "-" + 
                        data_array[i+14][2] + data_array[i+14][3] + "-" + 
                        data_array[i+14][4] + data_array[i+14][5])

                g.db.execute('insert into positions (imei, imsi, version, name, latitude, ' + 
                             ' longitude, date, time, value0, value1, value2, value3' +
                             ', value4, value5, value6, value7, value8, value9, value10)' +
                             ' values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?' + 
                             ', ?, ?, ?)',
                             [IMEI, IMSI, version, name, lat, lon, date, time, value0, 
                              value1, value2, value3, value4, value5, value6, value7,
                              value8, value9, value10])
                g.db.commit()
        #return true
        flash('New entry was successfully posted')
        return redirect(url_for('show_entries'))
        
    else:
        #return false
        flash('CRC check missmatch!')
        return redirect(url_for('show_entries'))


@app.route('/add', methods=['POST'])
def add_entry():
    #if not session.get('logged_in'):
    #    abort(401)
    apa = request.form['text'] + 'testdubbel'
    g.db.execute('insert into entries (title, text) values (?, ?)',
                 [request.form['title'], apa])
    g.db.commit()
    flash('New entry was successfully posted')
    return redirect(url_for('show_entries'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    error = None
    if request.method == 'POST':
        print(request.data)
        if request.form['username'] != app.config['USERNAME']:
            error = 'Invalid username'
        elif request.form['password'] != app.config['PASSWORD']:
            error = 'Invalid password'
        else:
            session['logged_in'] = True
            flash('You were logged in')
            return redirect(url_for('show_entries'))
    return render_template('login.html', error=error)

@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    flash('You were logged out')
    return redirect(url_for('show_entries'))



if __name__ == '__main__':
    app.run(host='pi1.lab.hummelgard.com', port=88, debug=True)

