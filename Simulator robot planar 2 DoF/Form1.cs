using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
/*Program by:fahmi mashuri
  source : https://github.com/fahmimsh/Trajektory-robot-planar-2dof
  created time : 10/12/2020 */
namespace Simulator_robot_planar_2_DoF
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }
        int Pilihan, Objek, endtime = 0, jalan, titik = 0, simpan_data_cek;
        double L1 = 4, L2 = 4, Teta1 = 0.0, Teta2 = 0.0, x1 = 4.0, x2 = 8.0, y1 = 0.0, y2 = 0.0;
        double x2_inv = 8.0, y2_inv = 0.0, x1_inv = 0.0, y1_inv = 0.0, runtime = 0;
        double inv_Teta1 = 0.0, inv_Teta2 = 0.0, rad1 = 0.0, rad2 = 0.0;
        double teta1start = 0.0, teta2start = 0.0, teta1end = 0.0, teta2end = 0.0;
        double y2_inv_end = 0.0, y2_inv_start = 0.0, x2_inv_start = 0.0, x2_inv_end = 0.0;
        double diameter = 0.0, sisi = 0.0, titik_X = 0.0, titik_Y = 0.0;
        string line; System.IO.StreamReader reader; StreamWriter csv;
        private void baca()
        {
            if (jalan == 2)
            {
                reader = new StreamReader("Robot planar.csv");
            }
        }
        /*--------FORWAWD KINEMATIKA----------*/
        private void Forward_kinematika()
        {
            x1 = L1 * Math.Cos(Teta1 * Math.PI / 180);
            y1 = L1 * Math.Sin(Teta1 * Math.PI / 180);

            x2 = x1 + L2 * Math.Cos((Teta1 + Teta2) * Math.PI / 180);
            y2 = y1 + L2 * Math.Sin((Teta1 + Teta2) * Math.PI / 180);

            textBox1.Text = Math.Round((decimal)x2, 2).ToString();
            textBox2.Text = Math.Round((decimal)y2, 2).ToString();

            double[] data1 = { x1, y1, x2, y2 }, data_point = { x2, y2 };
            drawArm(0, 1, data1);
            drawpoint(data_point);
        }
        /*---------INVERS KINEMATIKA-------------------*/
        private void Invers_kinematika()
        {
            if (Math.Sqrt(Math.Pow(x2_inv, 2) + Math.Pow(y2_inv, 2)) > L1 + L2 + 0.5)
            {
                timer1.Enabled = false;
                MessageBox.Show("Turunkan Nilai Posisi X atau Y !!!", "Batas Maksimum nilai");
                runtime = 0;
            }
            else if (Math.Sqrt(Math.Pow(x2_inv, 2) + Math.Pow(y2_inv, 2)) < (L1 - L2))
            {
                timer1.Enabled = false;
                MessageBox.Show("Naikkan Nilai Posisi X atau Y !!!", "Batas Minimum Nilai");
                runtime = 0;
            }
            else
            {
                if (x2_inv == 0) x2_inv = 0.00000000000001;
                else if (radioButton3.Checked == true)
                {
                    if (Math.Sqrt(Math.Pow(titik_X, 2) + Math.Pow(titik_Y, 2)) * (sisi * 0.5) > L1 + L2 || Math.Sqrt(Math.Pow(titik_X, 2) + Math.Pow(titik_Y, 2)) * (diameter * 0.5) > L1 + L2)
                    {
                        timer1.Enabled = false;
                        MessageBox.Show("Turunkan Sisi/diameter atau Titik tengah !!!", "Nilai Melebihi Batas");
                        runtime = 0;
                    }
                }
                rad2 = Math.Acos(((x2_inv * x2_inv) + (y2_inv * y2_inv) - (L1 * L1) - (L2 * L2)) / (2 * L1 * L2));
                rad1 = Math.Atan2(y2_inv, x2_inv) - Math.Atan2((L2 * Math.Sin(rad2)), (L1 + (L2 * Math.Cos(rad2))));
                inv_Teta1 = rad1 * 180 / Math.PI;
                inv_Teta2 = rad2 * 180 / Math.PI;

                x1_inv = L1 * Math.Cos(inv_Teta1 * Math.PI / 180);
                y1_inv = L1 * Math.Sin(inv_Teta1 * Math.PI / 180);

                textBox3.Text = Math.Round((decimal)inv_Teta1, 1).ToString();
                textBox4.Text = Math.Round((decimal)inv_Teta2, 1).ToString();

                double[] value2 = { x1_inv, y1_inv, x2_inv, y2_inv }, data_point = {x2_inv, y2_inv};
                drawArm(0, 1, value2); drawpoint(data_point);
            }
        }
        /*--------TRAYEKTORY-------------------------*/
        private void timer1_Tick(object sender, EventArgs e)
        {
            if (jalan == 1)
            {
                runtime += 0.25; titik += 1;
                if (Pilihan == 1)
                {
                    Teta1 = teta1start + (((teta1end - teta1start) * runtime) / endtime);
                    Teta2 = teta2start + (((teta2end - teta2start) * runtime) / endtime);
                    Forward_kinematika();
                    teta1start = Teta1;
                    teta2start = Teta2;
                } //forward
                else if (Pilihan == 2)
                {
                    x2_inv = x2_inv_start + (((x2_inv_end - x2_inv_start) * runtime) / endtime);
                    y2_inv = y2_inv_start + (((y2_inv_end - y2_inv_start) * runtime) / endtime);
                    Invers_kinematika();
                    x2_inv_start = x2_inv;
                    y2_inv_start = y2_inv;
                } //invers
                else if (Pilihan == 3)
                {
                    if (Objek == 1)
                    {
                        int i = 360 / endtime;
                        runtime += i;
                        x2_inv = titik_X + ((diameter / 2) * Math.Cos(Math.PI * runtime / 180));
                        y2_inv = titik_Y - ((diameter / 2) * Math.Sin(Math.PI * runtime / 180));
                        Invers_kinematika();
                    } //lingkaran 
                    else if (Objek == 2)
                    {
                        int i = Convert.ToInt32(sisi) / endtime;
                        runtime += i; double s = sisi / 2;
                        double x2_1 = titik_X - s, x2_2 = titik_X + s;
                        double y2_1 = titik_Y - s, y2_2 = titik_Y + s;
                        x2_inv = titik_X + runtime;
                        y2_inv = y2_1;
                        if (x2_2 < x2_inv)
                        {
                            x2_inv = x2_2;
                            y2_inv = (titik_Y - sisi) + runtime;
                            if (y2_inv > y2_2)
                            {
                                x2_inv = (titik_X + sisi * 2) - runtime;
                                y2_inv = y2_2;
                                if (x2_1 > x2_inv)
                                {
                                    x2_inv = x2_1;
                                    y2_inv = (titik_Y + sisi * 3) - runtime;
                                    if (y2_1 > y2_inv)
                                    {
                                        x2_inv = (titik_X - sisi * 4) + runtime;
                                        y2_inv = y2_1;
                                        if (titik_X < x2_inv)
                                        {
                                            timer1.Enabled = false;
                                            if (checkBox2.Checked == true)
                                            {
                                                MessageBox.Show("Data berhasil disimpan di file");
                                            }
                                            runtime = 0;
                                        }
                                    }
                                }
                            }
                        }
                        Invers_kinematika();
                    } //persegi
                    else if (Objek == 3)
                    {
                        int i = Convert.ToInt32(sisi) / endtime;
                        runtime += i; double s = sisi * 0.5;
                        x2_inv = titik_X + runtime;
                        y2_inv = titik_Y;
                        if (titik_X + sisi < x2_inv)
                        {
                            x2_inv = (titik_X + sisi * 2) - runtime;
                            y2_inv = (titik_Y - sisi) + runtime;
                            if (y2_inv > titik_Y + s)
                            {
                                x2_inv = (titik_X + sisi * 2) - runtime;
                                y2_inv = (titik_Y + sisi * 2) - runtime;
                                if (titik_Y >= y2_inv && titik_X >= x2_inv)
                                {
                                    timer1.Enabled = false;
                                    if (checkBox2.Checked == true)
                                    {
                                        MessageBox.Show("Data berhasil disimpan di file");
                                    }
                                    runtime = 0;

                                }
                            }
                        }
                        Invers_kinematika();
                    } //segitiga
                }
                if (runtime == endtime || runtime >= 360)
                {
                    if (checkBox4.Checked == false)
                    {
                        timer1.Enabled = false;
                        if (checkBox2.Checked == true)
                        {
                            MessageBox.Show("Data berhasil disimpan di file");
                        }
                        runtime = 0;
                    }
                    else
                        MessageBox.Show("Timer interval yang kamu masukkan kurang");
                }
                simpan_data();
            } else if (jalan == 2)
            {
                if ((line = reader.ReadLine()) != null)
                {
                    var values = line.Split(',');
                    titik = Convert.ToInt32(values[0]);
                    Console.WriteLine(x2_inv = Convert.ToDouble(values[1]));
                    Console.WriteLine(y2_inv = Convert.ToDouble(values[2]));
                    int milliseconds = 200;
                    Thread.Sleep(milliseconds);
                    Invers_kinematika();
                    Pilihan = 2;
                    simpan_data();
                }
                else
                {
                    reader.Close();
                    timer1.Enabled = false;
                    MessageBox.Show("Data CSV Sudah berhasil di PlayBack");
                }
            } // Play Back Data CSV
        }
        /*---------------INPUT NILAI------------------------*/
        private void chart1_MouseClick(object sender, MouseEventArgs e)
        {
            List<Point> points = new List<Point>();
            Point lastPoint = Point.Empty;
            lastPoint = e.Location;
            chart1.Invalidate();
            if (lastPoint != Point.Empty)
            {
                Pilihan = 2; titik += 1;
                x2_inv = chart1.ChartAreas[0].AxisX.PixelPositionToValue(lastPoint.X);
                y2_inv = chart1.ChartAreas[0].AxisY.PixelPositionToValue(lastPoint.Y);
                Invers_kinematika();
                simpan_data();
            }
            lastPoint = Point.Empty;
        }
        private void simpan_data()
        {
            if (Pilihan == 1)
            {
                label14.Text = Convert.ToString(Math.Round(runtime, 2));
                label15.Text = Convert.ToString(Math.Round(Teta1, 2));
                label17.Text = Convert.ToString(Math.Round(Teta2, 2));
                label18.Text = Convert.ToString(Math.Round(x2, 2));
                label19.Text = Convert.ToString(Math.Round(y2, 2));
            } else if (Pilihan == 2 || Pilihan == 3)
            {
                label14.Text = Convert.ToString(Math.Round(runtime, 2));
                label15.Text = Convert.ToString(Math.Round(inv_Teta1, 2));
                label17.Text = Convert.ToString(Math.Round(inv_Teta2, 2));
                label18.Text = Convert.ToString(Math.Round(x2_inv, 2));
                label19.Text = Convert.ToString(Math.Round(y2_inv, 2));
            }
            string s = "           ";
            listBox1.Items.Add(titik + s + label18.Text + s + label19.Text + s + label15.Text + s + label17.Text);
            listBox1.SelectedIndex = listBox1.Items.Count - 1;
            if (simpan_data_cek == 1)
            {
                csv = new StreamWriter("Robot planar.csv");
                var a = titik; var b = label18.Text; var c = label19.Text;
                var d = label15.Text; var f = label17.Text;
                var newLine = string.Format("{0},{1},{2},{3},{4}", a, b, c, d, f);
                listBox2.Items.Add(newLine);
                foreach (var baruLine in listBox2.Items)
                {
                    csv.WriteLine(baruLine);
                }
                csv.Close();
            } 
        }
        private void button1_Click(object sender, EventArgs e)
        {
            timer1.Enabled = true;
            if (radioButton1.Checked == false && radioButton2.Checked == false && radioButton3.Checked == false) 
            {
                timer1.Enabled = false;
                MessageBox.Show("Pilih Jenis simulasi gerak terlebih dahulu (INVERS, FORWARD ATAU OBJECT)", "SIMULASI GERAK BERKATA");
            } else if (endtime == 0)
            {
                timer1.Enabled = false;
                MessageBox.Show("Masukkan Timming(det) terlebih dahulu", "TIMER BERKATA");
                runtime = 0;
            }
            else
            {
                jalan = 1;
                if(radioButton3.Checked == true)
                {
                    if (checkBox3.Checked == false && checkBox4.Checked == false && checkBox5.Checked == false)
                    {
                        jalan = 0; timer1.Enabled = false;
                        MessageBox.Show("Pilih Objek Trajektory terlebih dahulu (Lingkaran, Persegi ATAU Segitiga)", "Objek Trayektori BERKATA");
                    } else if (checkBox3.Checked == true) { endtime *= 4; }
                    else if (checkBox4.Checked == true) { endtime += 20; }
                    else if(checkBox5.Checked == true) { endtime += 10; }
                }
            }
        }
        private void button2_Click(object sender, EventArgs e)
        {
            timer1.Enabled = true; 
            simpan_data_cek = 0;
            jalan = 2; baca(); line = "1";
        }
        private void button3_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            Teta1 = 0.0; Teta2 = 0.0; runtime = 0; x2_inv = L1 + L2; y2_inv = 0.0;
            label14.Text = "0"; label15.Text = "0"; label17.Text = "0";
            label18.Text = "0"; label19.Text = "0"; textBox5.Text = "0";
            listBox1.Items.Clear(); checkBox2.Enabled = false;
            Forward_kinematika(); Invers_kinematika();
            chart1.Series[2].Points.Clear(); checkBox2.Enabled = true;
        }
        private void textBox3_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox3.Text, out teta1end);
            if (double.TryParse(textBox3.Text, out teta1end))
            {
                Console.WriteLine(teta1end);
            }
            Console.ReadLine();
        }
        private void textBox4_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox4.Text, out teta2end);
            if (double.TryParse(textBox4.Text, out teta2end))
            {
                Console.WriteLine(teta2end);
            }
            Console.ReadLine();
        }
        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox1.Text, out x2_inv_end);
            if (double.TryParse(textBox1.Text, out x2_inv_end))
            {
                Console.WriteLine(x2_inv_end);
            }  Console.ReadLine();
        }
        private void textBox2_TextChanged(object sender, EventArgs e)
        {   double.TryParse(textBox2.Text, out y2_inv_end);
            if (double.TryParse(textBox2.Text, out y2_inv_end))
            {
                Console.WriteLine(y2_inv_end);
            }
            Console.ReadLine();
        }
        private void textBox5_TextChanged(object sender, EventArgs e)
        {
            int.TryParse(textBox5.Text, out endtime);
            if (int.TryParse(textBox5.Text, out endtime))
            {
                Console.WriteLine(endtime);
            }
            Console.ReadLine();
        }
        private void textBox6_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox6.Text, out diameter);
            if (double.TryParse(textBox6.Text, out diameter))
            {
                Console.WriteLine(diameter);
            }
            Console.ReadLine();
        }
        private void textBox7_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox7.Text, out sisi);
            if (double.TryParse(textBox7.Text, out sisi))
            {
                Console.WriteLine(sisi);
            }
            Console.ReadLine();
        }
        private void textBox8_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox8.Text, out titik_X);
            if (double.TryParse(textBox8.Text, out titik_X))
            {
                Console.WriteLine(titik_X);
            }
            Console.ReadLine();
        }
        private void textBox9_TextChanged(object sender, EventArgs e)
        {
            double.TryParse(textBox9.Text, out titik_Y);
            if (double.TryParse(textBox9.Text, out titik_Y))
            {
                Console.WriteLine(titik_Y);
            }
            Console.ReadLine();
        }
        //-------------------------------setting----------------------------//
        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButton1.Checked)
            {
                Pilihan = 1;
                groupBox3.Enabled = false; groupBox2.Enabled = true;
                groupBox1.Enabled = false;
            }
            else
            {
                groupBox2.Enabled = false;
            }
        }
        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButton2.Checked)
            {
                Pilihan = 2;
                groupBox3.Enabled = true; groupBox2.Enabled = false;
                groupBox1.Enabled = false;
            }
            else
            {
                groupBox3.Enabled = false;
            }
        }
        private void radioButton3_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButton3.Checked)
            {
                Pilihan = 3;
                groupBox3.Enabled = false; groupBox2.Enabled = false;
                groupBox1.Enabled = true;
            }
            else
            {
                groupBox1.Enabled = false;
            }
        }
        private void drawArm(int line, int dot, double[] data)
        {
            chart1.Series[line].Points[1].XValue = data[0];
            chart1.Series[line].Points[1].YValues[0] = data[1];
            chart1.Series[dot].Points[1].XValue = data[0];
            chart1.Series[dot].Points[1].YValues[0] = data[1];

            chart1.Series[line].Points[2].XValue = data[2];
            chart1.Series[line].Points[2].YValues[0] = data[3];
            chart1.Series[dot].Points[2].XValue = data[2];
            chart1.Series[dot].Points[2].YValues[0] = data[3];
        }
        private void drawpoint(double[] data_poit)
        {
            chart1.Series[2].Points.AddXY(data_poit[0],data_poit[1]);
        }
        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox1.Checked == true)
            {
                chart1.ChartAreas[0].AxisX.MinorGrid.Enabled = true;
                chart1.ChartAreas[0].AxisY.MinorGrid.Enabled = true;
            }
            else
            {
                chart1.ChartAreas[0].AxisX.MinorGrid.Enabled = false;
                chart1.ChartAreas[0].AxisY.MinorGrid.Enabled = false;
            }
        }
        private void checkBox2_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox2.Checked == true)
            {
                simpan_data_cek = 1;
            }
        }
        private void checkBox3_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox3.Checked == true)
            {
                checkBox4.Checked = false; checkBox5.Checked = false;
                textBox7.Enabled = false; textBox6.Enabled = true;
                Objek = 1; label27.Text = "Titik Tengah";
            }
        }
        private void checkBox4_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox4.Checked == true)
            {
                checkBox3.Checked = false;checkBox5.Checked = false;
                textBox6.Enabled = false; textBox7.Enabled = true;
                Objek = 2; label27.Text = "Titik Tengah";
            }
        }
        private void checkBox5_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox5.Checked == true)
            {
                checkBox3.Checked = false; checkBox4.Checked = false;
                textBox6.Enabled = false; textBox7.Enabled = true;
                Objek = 3; label27.Text = "Titik Ujung Kiri";
            }
        }
        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            if ((double)numericUpDown1.Value + L2 > 8)
            {
                MessageBox.Show("TURUNKAN NILAI Panjang Lengan 2 !!", "Batas maksimum nilai");
                numericUpDown1.Value = (decimal)L1;
                timer1.Enabled = false;
                runtime = 0;
            } else{
                L1 = (double)numericUpDown1.Value;
                Forward_kinematika();
                double[] data1 = { x1, y1, x2, y2 };
                drawArm(0, 1, data1);
            }
        }
        private void numericUpDown2_ValueChanged(object sender, EventArgs e)
        {
            if ((double)numericUpDown2.Value + L1 > 8)
            {
                MessageBox.Show("TURUNKAN NILAI Panjang lengan 1  !!!", "Batas Maksimum nilai");  
                numericUpDown2.Value = (decimal)L2;
                timer1.Enabled = false;
                runtime = 0;
            } else{
                L2 = (double)numericUpDown2.Value;
                Forward_kinematika();
                double[] data1 = { x1, y1, x2, y2 };
                drawArm(0, 1, data1);
            }
        }
        private void Form_Paint(object sender, PaintEventArgs e)
        {
            Graphics l = e.Graphics;
            Pen p = new Pen(Color.Red, 2);
            ChartArea ca = chart1.ChartAreas[0];
            Axis ax = ca.AxisX;
            Axis ay = ca.AxisY;
            Series s = chart1.Series[0];
            DataPoint dpCenter = s.Points[0];

            double xVal = dpCenter.XValue;
            double yVal = dpCenter.YValues[0];

            double rad1 = L1 + L2;
            double rad2 = L1 - L2;

            float xRad1 = (float)(ax.ValueToPixelPosition(0) - ax.ValueToPixelPosition(rad1));
            float yRad1 = (float)(ay.ValueToPixelPosition(0) - ay.ValueToPixelPosition(rad1));
            float xRad2 = (float)(ax.ValueToPixelPosition(0) - ax.ValueToPixelPosition(rad2));
            float yRad2 = (float)(ay.ValueToPixelPosition(0) - ay.ValueToPixelPosition(rad2));

            float xc = (float)ax.ValueToPixelPosition(xVal);
            float yc = (float)ay.ValueToPixelPosition(yVal);

            Rectangle r1 = Rectangle.Round(new RectangleF(xc - xRad1, yc - yRad1, xRad1 * 2, yRad1 * 2));
            Rectangle r2 = Rectangle.Round(new RectangleF(xc - xRad2, yc - yRad2, xRad2 * 2, yRad2 * 2));

            l.DrawEllipse(p, r1);
            l.DrawEllipse(p, r2);
        }
    }
}

