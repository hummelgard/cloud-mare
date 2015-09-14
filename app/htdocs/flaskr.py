# all the imports
import sqlite3, crc16
from flask import Flask, request, session, g, redirect, url_for, \
     abort, render_template, flash

# configuration
DATABASE = '/tmp/flaskr.db'
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

@app.route('/tempmap')
def show_tempmap():
    cur = g.db.execute('select latitude, longitude, value2 from positions ' )
    posData=[] 
    for row in cur.fetchall():
      posData.append(row)
    
    dataTransp = list(map(list, zip(*posData)))
    
    #convert strings to float, int
    latitude = list(map(float, dataTransp[0]))
    longitude = list(map(float, dataTransp[1]))	
    value2 = list(map(int, dataTransp[2]))

    #latitude width
    width = int( (max(latitude)-min(latitude) )*10000)
      
    #longitude width
    height = int( (max(longitude)-min(longitude) )*10000)
    
    #ortsjon
    latCenter = 6216576
    lonCenter = 1717866

    density = 1
    print(width)
    print(height)
    print(int(min(latitude)*10000))

    print(int(max(latitude)*10000))
    #for x in range(int(min(latitude)*10000), int(max(latitude)*10000), density):
    #  for y in range(int(min(longitude)*10000), int(max(longitude)*10000), density):

    points=[]    
    
    for y in range(latCenter-50,latCenter+50,5):
      for x in range(lonCenter-150,lonCenter+50,10):
        avg_count=1
        temp=0
        for i in range(len(posData)):
         
          x0 = int(float(posData[i][1])*100000)
          y0 = int(float(posData[i][0])*100000)
           
          if( ( abs(x-x0) + abs(y-y0) ) < 3 ):
            #print("yes")
            temp = temp + (value2[i]-15)*5
            avg_count = avg_count + 1
        temp = temp / avg_count
        if(temp != 0):
       	  points.append( dict(latitude=str(y/100000.0),longitude=str(x/100000.0),value2=str(temp) ) )

    return render_template('show_tempmap.html',points=points)


@app.route('/heatmap')
def show_heatmap():
    cur = g.db.execute('select latitude, longitude from positions ' )
    points = [dict(latitude=row[0], longitude=row[1]) 
              for row in cur.fetchall()]
    return render_template('show_heatmap.html',points=points)

@app.route('/gps')
def show_gps_entries():
    cur = g.db.execute('select IMEI_id, latitude, longitude, date, time,' +
                       ' value1, value2, value3 from positions order by id desc')
    positions = [dict(IMEI_id=row[0], latitude=row[1], longitude=row[2], 
                 date=row[3], time=row[4], value1=row[5], 
                 value2=row[6], value3=row[7]) for row in 
cur.fetchall()]
    return render_template('show_gps_entries.html', positions=positions)


@app.route('/addData', methods=['POST'])
def addData_entry():
    #if not session.get('logged_in'):
    #    abort(401)

    IMEI = request.form['IMEI']
    data = request.form['data']
    sum = int(request.form['sum'])
    check_string ="IMEI=" + IMEI + "&data=" + data + "&sum="
    check_sum = ''.join(format(ord(x), 'b') for x in check_string).count('1')
    data_array = data.split('#')
    length = len(data_array)    
   
    if check_sum == sum:
       
        for i in range(0,length,7):
            #if version == 1:
                lat = data_array[i]
                lon = data_array[i+1]

                date = (data_array[i+2][0] + data_array[i+2][1] + "-" + 
                        data_array[i+2][2] + data_array[i+2][3] + "-" + 
                        data_array[i+2][4] + data_array[i+2][5])

                time = (data_array[i+3][0] + data_array[i+3][1] + ":" +
                        data_array[i+3][2] + data_array[i+3][3] + ":" +
                        data_array[i+3][4] + data_array[i+3][5])
  
                value1 = data_array[i+4]
                value2 = data_array[i+5]
                value3 = data_array[i+6]
 
                g.db.execute('insert into positions (IMEI_id,latitude,' + 
                             ' longitude, date, time, value1, value2, value3)' +
                             ' values (?, ?, ?, ?, ?, ?, ?, ?)',[IMEI, 
                             lat, lon, date, time, value1, value2, value3])
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

