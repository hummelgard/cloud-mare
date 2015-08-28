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


@app.route('/heatmap')
def show_heatmap():
    cur = g.db.execute('select latitude, longitude from positions ' + 
                       'where value2 LIKE "GPS"' )
    points = [dict(latitude=row[0], longitude=row[1]) 
              for row in cur.fetchall()]
    return render_template('show_heatmap.html',points=points)

@app.route('/gps')
def show_gps_entries():
    cur = g.db.execute('select IMEI_id, latitude, longitude, date, time,' +
                       ' value1, value2 from positions order by id desc')
    positions = [dict(IMEI_id=row[0], latitude=row[1], longitude=row[2], 
                 date=row[3], time=row[4], value1=row[5], 
                 value2=row[6]) for row in cur.fetchall()]
    return render_template('show_gps_entries.html', positions=positions)


@app.route('/addData', methods=['POST'])
def addData_entry():
    #if not session.get('logged_in'):
    #    abort(401)
    IMEI_id = request.form['IMEI_id']
    data_buffer = request.form['data']
    part1, part2, part3, part4 = data_buffer.split('#',3)
    crc = int(part1)
    version = int(part2)
    length = int(part3)
    data_string = part4
    data_array = data_string.split('#')
    if crc == crc16.crc16xmodem(bytes(data_buffer.split('#',1)[1],'UTF-8')):

        for i in range(0,int(length)*6,6):
            if version == 1:
                lat = data_array[i]
                lon = data_array[i+1]
                date = data_array[i+2]
                time = data_array[i+3].split(' ')[0]
                value1 = data_array[i+4]
                value2 = data_array[i+5]

                g.db.execute('insert into positions (IMEI_id,latitude,' + 
                             ' longitude, date, time, value1, value2)' +
                             ' values (?, ?, ?, ?, ?, ?, ?)',[IMEI_id, 
                             lat, lon, date, time, value1, value2])
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

