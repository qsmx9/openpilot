#include "selfdrive/ui/qt/widgets/drive_stats_panel.h"

#include <algorithm>
#include <QFile>
#include <QJsonDocument>
#include <QStorageInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QRegExp>

// ---------------- BarChart ----------------
BarChart::BarChart(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(170);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void BarChart::setData(const QJsonObject& daily) {
  m_daily = daily;
  update();
}

void BarChart::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QRect r = contentsRect();
  QStringList days = m_daily.keys();
  std::sort(days.begin(), days.end());
  while (days.size() > 7) days.removeFirst();
  if (days.isEmpty()) {
    p.setPen(QColor("#888888"));
    p.setFont(QFont("DejaVu Sans", 16));
    p.drawText(r, Qt::AlignCenter, "暂无数据");
    return;
  }
  double maxv = 0.0001;
  for (const QString& d : days) maxv = qMax(maxv, m_daily[d].toDouble());

  int n = days.size();
  int gap = 14;
  int bw = qMax(6, (r.width() - gap * (n + 1)) / n);
  int baseY = r.bottom() - 20;
  int top = r.top() + 24;
  int maxH = baseY - top;
  QColor bar("#FF9A00");
  QColor bg("#2A2A2A");
  for (int i = 0; i < n; ++i) {
    double v = m_daily[days[i]].toDouble();
    int x = r.left() + gap + i * (bw + gap);
    int h = int((v / maxv) * maxH);
    if (h < 2) h = 2;
    p.setPen(Qt::NoPen);
    p.setBrush(bg);
    p.drawRoundedRect(x, top, bw, maxH, 4, 4);
    p.setBrush(bar);
    p.drawRoundedRect(x, baseY - h, bw, h, 4, 4);
    p.setPen(QColor("#CCCCCC"));
    p.setFont(QFont("DejaVu Sans", 11));
    p.drawText(QRect(x - 10, baseY - h - 16, bw + 20, 14), Qt::AlignCenter, QString::number(v, 'f', 1));
    p.drawText(QRect(x - 10, baseY + 2, bw + 20, 14), Qt::AlignCenter, days[i].mid(5).replace("-", "/"));
  }
}

// ---------------- DriveStatsPanel ----------------
DriveStatsPanel::DriveStatsPanel(QWidget* parent) : QFrame(parent) {
  QHBoxLayout* root = new QHBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(30);

  // left: table
  QFrame* left = new QFrame(this);
  QVBoxLayout* lv = new QVBoxLayout(left);
  lv->setContentsMargins(0, 0, 0, 0);
  lv->setSpacing(18);

  QLabel* title = new QLabel("驾驶数据");
  title->setStyleSheet("font-size: 34px; font-weight: 600; color: white;");
  lv->addWidget(title);

  QGridLayout* grid = new QGridLayout();
  grid->setHorizontalSpacing(30);
  grid->setVerticalSpacing(14);
  QStringList headers = {"", "行程(次)", "里程(km)", "时长(min)"};
  for (int c = 0; c < headers.size(); ++c) {
    QLabel* h = new QLabel(headers[c]);
    h->setStyleSheet("font-size: 20px; color: #AAAAAA;");
    grid->addWidget(h, 0, c);
  }
  int row = 1;
  auto addRow = [&](const QString& name, QLabel*& trips, QLabel*& km, QLabel*& min) {
    QLabel* nm = new QLabel(name);
    nm->setStyleSheet("font-size: 23px; color: white;");
    grid->addWidget(nm, row, 0);
    trips = ledLabel("0"); km = ledLabel("0"); min = ledLabel("0");
    grid->addWidget(trips, row, 1);
    grid->addWidget(km, row, 2);
    grid->addWidget(min, row, 3);
    row++;
  };
  addRow("今日", v_today_trips, v_today_km, v_today_min);
  addRow("本周", v_week_trips, v_week_km, v_week_min);
  addRow("累计", v_total_trips, v_total_km, v_total_min);
  lv->addLayout(grid);
  lv->addStretch();
  root->addWidget(left, 3);

  // right: two cards
  QVBoxLayout* rv = new QVBoxLayout();
  rv->setContentsMargins(0, 0, 0, 0);
  rv->setSpacing(30);

  barChart = new BarChart(this);
  rv->addWidget(card("最近 7 天里程 (km)", barChart), 1);

  // system status card
  QFrame* sysBody = new QFrame(this);
  QGridLayout* sg = new QGridLayout(sysBody);
  sg->setHorizontalSpacing(20);
  sg->setVerticalSpacing(12);
  s_cpu = new QLabel("--"); s_mem = new QLabel("--"); s_store = new QLabel("--"); s_uptime = new QLabel("--");
  int sr = 0;
  auto sysRow = [&](const QString& k, QLabel* v) {
    QLabel* kl = new QLabel(k);
    kl->setStyleSheet("font-size: 19px; color: #AAAAAA;");
    v->setStyleSheet("font-size: 23px; color: #FF9A00; font-weight: 600;");
    sg->addWidget(kl, sr, 0);
    sg->addWidget(v, sr, 1);
    sr++;
  };
  sysRow("CPU 温度", s_cpu);
  sysRow("内存可用", s_mem);
  sysRow("存储剩余", s_store);
  sysRow("运行时间", s_uptime);
  rv->addWidget(card("系统状态", sysBody), 1);

  root->addLayout(rv, 2);

  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &DriveStatsPanel::refresh);
  timer->start(5000);
  refresh();
}

QLabel* DriveStatsPanel::ledLabel(const QString& text) {
  QLabel* l = new QLabel(text);
  l->setStyleSheet("font-size: 27px; font-weight: 600; color: #FF9A00;");
  l->setAlignment(Qt::AlignRight);
  return l;
}

QFrame* DriveStatsPanel::card(const QString& title, QWidget* body) {
  QFrame* c = new QFrame(this);
  c->setObjectName("card");
  c->setStyleSheet("#card { border-radius: 10px; background-color: #1E1E1E; }");
  QVBoxLayout* l = new QVBoxLayout(c);
  l->setContentsMargins(22, 18, 22, 18);
  l->setSpacing(14);
  QLabel* t = new QLabel(title);
  t->setStyleSheet("font-size: 22px; font-weight: 600; color: white;");
  l->addWidget(t);
  l->addWidget(body, 1);
  return c;
}

QString DriveStatsPanel::readFile(const QString& path) {
  QFile f(path);
  if (f.open(QIODevice::ReadOnly)) return QString::fromUtf8(f.readAll()).trimmed();
  return "";
}

void DriveStatsPanel::refresh() {
  QString raw = QString::fromStdString(params.get("DriveStats"));
  QJsonObject obj;
  if (!raw.isEmpty()) {
    QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8());
    if (doc.isObject()) obj = doc.object();
  }
  auto bucket = [&](const char* key, QLabel* trips, QLabel* km, QLabel* min) {
    QJsonObject b = obj.value(key).toObject();
    trips->setText(QString::number(int(b.value("trips").toDouble())));
    km->setText(QString::number(b.value("km").toDouble(), 'f', 1));
    min->setText(QString::number(b.value("min").toDouble(), 'f', 1));
  };
  bucket("today", v_today_trips, v_today_km, v_today_min);
  bucket("week", v_week_trips, v_week_km, v_week_min);
  bucket("total", v_total_trips, v_total_km, v_total_min);
  barChart->setData(obj.value("daily").toObject());

  // system
  QString t = readFile("/sys/class/thermal/thermal_zone0/temp");
  if (!t.isEmpty()) s_cpu->setText(QString::number(t.toInt() / 1000) + " °C");

  QString mem = readFile("/proc/meminfo");
  double mt = 0, ma = 0;
  for (const QString& line : mem.split("\n")) {
    QStringList parts = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
    if (parts.size() >= 2) {
      if (line.startsWith("MemTotal:")) mt = parts[1].toDouble();
      else if (line.startsWith("MemAvailable:")) ma = parts[1].toDouble();
    }
  }
  if (mt > 0) s_mem->setText(QString::number(int(ma / mt * 100)) + " %");

  QStorageInfo si("/data");
  if (si.isValid()) {
    double gb = si.bytesAvailable() / 1024.0 / 1024.0 / 1024.0;
    s_store->setText(QString::number(gb, 'f', 1) + " GB");
  }

  QString up = readFile("/proc/uptime");
  QStringList upParts = up.split(" ");
  if (!upParts.isEmpty()) {
    double secs = upParts[0].toDouble();
    int d = int(secs / 86400); secs -= d * 86400;
    int h = int(secs / 3600); secs -= h * 3600;
    int m = int(secs / 60);
    s_uptime->setText(QString("%1天%2时%3分").arg(d).arg(h).arg(m));
  }
}
