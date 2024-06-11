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

    def create_user(self, user):
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
                return cur.fetchall() 
        except Error as e:
            print(e)
            return None
    
    def insert_quadrant(self, quadrant):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''INSERT INTO Quadrant (quadrant_position, quadrant_status, reboiler_id) VALUES (?, ?, ?)'''
                cur = conn.cursor()
                cur.execute(sql, (quadrant.quadrant_position, quadrant.quadrant_status, quadrant.reboiler_id))
                conn.commit()
                return cur.lastrowid
        except Error as e:
            print(e)
            return None

    def get_quadrant(self, quadrant_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Quadrant WHERE quadrand_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (quadrant_id,))
                return cur.fetchone()
        except Error as e:
            print(e)
            return None

    def update_quadrant(self, quadrant_id, quadrant):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''UPDATE Quadrant SET quadrant_position = ?, quadrant_status = ?, reboiler_id = ? WHERE quadrant_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (quadrant.quadrant_position, quadrant.quadrant_status, quadrant.reboiler_id, quadrant_id))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def delete_quadrant(self, quadrant_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''DELETE FROM Quadrant WHERE quadrant_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (quadrant_id,))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def get_all_quadrants(self):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Quadrant'''
                cur = conn.cursor()
                cur.execute(sql)
                return cur.fetchall()
        except Error as e:
            print(e)
            return None
    
    def insert_quadrant_zone(self, quadrant_zone):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''INSERT INTO Quadrant_Zone (zone_area, zone_status, quadrant_id) VALUES (?, ?, ?)'''
                cur = conn.cursor()
                cur.execute(sql, (quadrant_zone.zone_area, quadrant_zone.zone_status, quadrant_zone.quadrant_id))
                conn.commit()
                return cur.lastrowid
        except Error as e:
            print(e)
            return None

    def get_quadrant_zone(self, zone_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Quadrant_Zone WHERE zone_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (zone_id,))
                return cur.fetchone()
        except Error as e:
            print(e)
            return None

    def update_quadrant_zone(self, zone_id, quadrant_zone):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''UPDATE Quadrant_Zone SET zone_area = ?, zone_status = ?, quadrant_id = ? WHERE zone_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (quadrant_zone.zone_area, quadrant_zone.zone_status, quadrant_zone.quadrant_id, zone_id))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def delete_quadrant_zone(self, zone_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''DELETE FROM Quadrant_Zone WHERE zone_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (zone_id,))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def get_all_quadrant_zones(self):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Quadrant_Zone'''
                cur = conn.cursor()
                cur.execute(sql)
                return cur.fetchall()
        except Error as e:
            print(e)
            return None

    def insert_reboiler(self, reboiler):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''INSERT INTO Reboilers (reboiler_num_pipes, reboiler_status, refinary_id) VALUES (?, ?, ?)'''
                cur = conn.cursor()
                cur.execute(sql, (reboiler.num_pipes, reboiler.status, reboiler.refinery_id))
                conn.commit()
                return cur.lastrowid
        except Error as e:
            print(e)
            return None

    def get_reboiler(self, reboiler_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Reboilers WHERE reboiler_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (reboiler_id,))
                return cur.fetchone()
        except Error as e:
            print(e)
            return None

    def update_reboiler(self, reboiler_id, reboiler):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''UPDATE Reboilers SET num_pipes = ?, status = ?, refinery_id = ? WHERE reboiler_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (reboiler.num_pipes, reboiler.status, reboiler.refinery_id, reboiler_id))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def delete_reboiler(self, reboiler_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''DELETE FROM Reboilers WHERE reboiler_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (reboiler_id,))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def get_all_reboilers(self):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Reboilers'''
                cur = conn.cursor()
                cur.execute(sql)
                return cur.fetchall()
        except Error as e:
            print(e)
            return None

    def insert_refinary(self, refinary):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''INSERT INTO Refinary (refinary_location, refinary_num_reboilers, refinary_name) VALUES (?, ?, ?)'''
                cur = conn.cursor()
                cur.execute(sql, (refinary.location, refinary.num_reboilers, refinary.name))
                conn.commit()
                return cur.lastrowid
        except Error as e:
            print(e)
            return None

    def get_refinary(self, refinary_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Refinary WHERE refinary_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (refinary_id,))
                return cur.fetchone()
        except Error as e:
            print(e)
            return None

    def update_refinary(self, refinary_id, refinary):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''UPDATE Refinary SET refinary_location = ?, refinary_num_reboilers = ?, refinary_name = ? WHERE refinary_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (refinary.location, refinary.num_reboilers, refinary.name, refinary_id))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def delete_refinary(self, refinary_id):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''DELETE FROM Refinary WHERE refinary_id = ?'''
                cur = conn.cursor()
                cur.execute(sql, (refinary_id,))
                conn.commit()
                return cur.rowcount
        except Error as e:
            print(e)
            return None

    def get_all_refinaries(self):
        conn = self.create_connection()
        try:
            with conn:
                sql = '''SELECT * FROM Refinary'''
                cur = conn.cursor()
                cur.execute(sql)
                return cur.fetchall()
        except Error as e:
            print(e)
            return None