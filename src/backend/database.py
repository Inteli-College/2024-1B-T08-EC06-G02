import sqlite3
from sqlite3 import Error

class DatabaseConnector:
    def __init__(self, path):
        self.path = path

    def create_connection(self):
        conn = None
        try:
            conn = sqlite3.connect(self.path)
        except Error as e:
            print(e)
        return conn

    def insert_in_table(self, username):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''INSERT INTO users(username) VALUES(?)'''
                cur = conn.cursor()
                cur.execute(sql, (username,))
                conn.commit()
                return cur.lastrowid
        except Error as e:
            print(e)
            return None

    def get_user(self, username):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM users WHERE username=?'''
                cur = conn.cursor()
                cur.execute(sql, (username,))
                row = cur.fetchone()
                return row
        except Error as e:
            print(e)
            return None

    def update_user(self, username):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''UPDATE users SET username=? WHERE username=?'''
                cur = conn.cursor()
                cur.execute(sql, (username, username))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def delete_user(self, username):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''DELETE FROM users WHERE username=?'''
                cur = conn.cursor()
                cur.execute(sql, (username,))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None
