drop table if exists entries;
create table entries (
  id integer primary key autoincrement,
  title text not null,
  text text not null
);

drop table if exists positions;
create table positions (
  id integer primary key autoincrement,
  imei text not null,
  name text not null,
  latitude text not null,
  longitude text not null,
  date text not null,
  time text not null,
  value0 text not null,
  value1 text not null,
  value2 text not null,
  value3 text not null,
  value4 text not null,
  value5 text not null,
  value6 text not null,
  value7 text not null,
  value8 text not null,
  value9 text not null,
  value10 text not null


);


