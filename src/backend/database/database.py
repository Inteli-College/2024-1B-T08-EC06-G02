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

    def insert_in_table(self, user):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''INSERT INTO Users(user_email, user_password, user_role, user_name) VALUES(?,?,?,?)'''
                cur = conn.cursor()
                cur.execute(sql, (user.user_email, user.user_password, user.user_role, user.user_name))
                conn.commit()
                return cur.lastrowid
        except Error as e:
            print(e)
            return None

    def get_user(self, user_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Users WHERE user_id=?'''
                cur = conn.cursor()
                cur.execute(sql, (user_id,))
                return cur.fetchone()
        except Error as e:
            print(e)
            return None

    def update_user(self, user_id, user):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''UPDATE Users SET user_email=?, user_password=?, user_role=?, user_name=? WHERE user_id=?'''
                cur = conn.cursor()
                cur.execute(sql, (user.user_email, user.user_password, user.user_role, user.user_name, user_id))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def delete_user(self, user_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''DELETE FROM Users WHERE user_id=?'''
                cur = conn.cursor()
                cur.execute(sql, (user_id,))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def get_all_users(self):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Users'''
                cur = conn.cursor()
                cur.execute(sql)
                return cur.fetchall()  # Retorna todos os usuários
        except Error as e:
            print(e)
            return None
