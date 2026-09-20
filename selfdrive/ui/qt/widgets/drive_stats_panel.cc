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
  setMinimumHeight(280);
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
    p.setFont(QFont("DejaVu Sans", 30));
    p.drawText(r, Qt::AlignCenter, "暂无数据");
    return;
  }
  double maxv = 0.0001;
  for (const QString& d : days) maxv = qMax(maxv, m_daily[d].toDouble());
  int n = days.size();
  int gap = 22;
  int bw = qMax(12, (r.width() - gap * (n + 1)) / n);
  int top = r.top() + 44;
  int baseY = r.bottom() - 38;
  int maxH = baseY - top;
  QColor bar("#FF9A00");
  QColor bg("#2A2A2A");
  for (int i = 0; i < n; ++i) {
    double v = m_daily[days[i]].toDouble();
    int x = r.left() + gap + i * (bw + gap);
    int h = int((v / maxv) * maxH);
    if (h < 3) h = 3;
    p.setPen(Qt::NoPen);
    p.setBrush(bg);
    p.drawRoundedRect(x, top, bw, maxH, 6, 6);
    p.setBrush(bar);
    p.drawRoundedRect(x, baseY - h, bw, h, 6, 6);
    p.setPen(QColor("#E8E8E8"));
    p.setFont(QFont("DejaVu Sans", 22));
    p.drawText(QRect(x - 20, baseY - h - 32, bw + 40, 28), Qt::AlignCenter, QString::number(v, 'f', 1));
    p.drawText(QRect(x - 20, baseY + 6, bw + 40, 24), Qt::AlignCenter, days[i].mid(5).replace("-", "/"));
  }
}

// ---------------- DriveStatsPanel ----------------
DriveStatsPanel::DriveStatsPanel(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* root = new QVBoxLayout(this);
  root->setContentsMargins(40, 26, 40, 26);
  root->setSpacing(26);

  QLabel* title = new QLabel("驾驶数据");
  title->setStyleSheet("font-size: 58px; font-weight: 700; color: white;");
  root->addWidget(title);

  // hero row: 3 big mileage blocks
  QHBoxLayout* hero = new QHBoxLayout();
  hero->setSpacing(22);
  auto heroBlock = [&](const QString& name, QLabel*& km, QLabel*& sub) -> QFrame* {
    QFrame* b = new QFrame(this);
    b->setObjectName("hero");
    b->setStyleSheet("#hero{border-radius:14px;background-color:#161616;}");
    QVBoxLayout* bl = new QVBoxLayout(b);
    bl->setContentsMargins(16, 18, 16, 18);
    bl->setSpacing(8);
    QLabel* nm = new QLabel(name);
    nm->setStyleSheet("font-size: 36px; color:#AAAAAA;");
    nm->setAlignment(Qt::AlignCenter);
    bl->addWidget(nm);
    km = new QLabel("0.0");
    km->setStyleSheet("font-size: 84px; font-weight:800; color:#FF9A00;");
    km->setAlignment(Qt::AlignCenter);
    bl->addWidget(km);
    QLabel* unit = new QLabel("km");
    unit->setStyleSheet("font-size: 32px; color:#FF9A00;");
    unit->setAlignment(Qt::AlignCenter);
    bl->addWidget(unit);
    sub = new QLabel("行程 0 次 · 时长 0 分");
    sub->setStyleSheet("font-size: 26px; color:#CCCCCC;");
    sub->setAlignment(Qt::AlignCenter);
    bl->addWidget(sub);
    return b;
  };
  hero->addWidget(heroBlock("今日", v_today_km, v_today_sub), 1);
  hero->addWidget(heroBlock("本周", v_week_km, v_week_sub), 1);
  hero->addWidget(heroBlock("累计", v_total_km, v_total_sub), 1);
  root->addLayout(hero);

  // bottom row: chart (left, wider) + system status (right)
  QHBoxLayout* bottom = new QHBoxLayout();
  bottom->setSpacing(22);
  barChart = new BarChart(this);
  bottom->addWidget(card("最近 7 天里程 (km)", barChart), 3);

  QFrame* sysBody = new QFrame(this);
  QGridLayout* sg = new QGridLayout(sysBody);
  sg->setHorizontalSpacing(18);
  sg->setVerticalSpacing(18);
  auto tile = [&](const QString& k, QLabel*& v) -> QFrame* {
    QFrame* t = new QFrame(this);
    t->setObjectName("tile");
    t->setStyleSheet("#tile{border-radius:10px;background-color:#1A1A1A;}");
    QVBoxLayout* tl = new QVBoxLayout(t);
    tl->setContentsMargins(16, 14, 16, 14);
    tl->setSpacing(8);
    QLabel* kl = new QLabel(k);
    kl->setStyleSheet("font-size: 28px; color:#AAAAAA;");
    kl->setAlignment(Qt::AlignCenter);
    tl->addWidget(kl);
    v = new QLabel("--");
    v->setStyleSheet("font-size: 42px; font-weight:700; color:#FF9A00;");
    v->setAlignment(Qt::AlignCenter);
    tl->addWidget(v);
    return t;
  };
  s_cpu = nullptr; s_mem = nullptr; s_store = nullptr; s_uptime = nullptr;
  sg->addWidget(tile("CPU 温度", s_cpu), 0, 0);
  sg->addWidget(tile("内存可用", s_mem), 0, 1);
  sg->addWidget(tile("存储剩余", s_store), 1, 0);
  sg->addWidget(tile("运行时间", s_uptime), 1, 1);
  bottom->addWidget(card("系统状态", sysBody), 2);

  root->addLayout(bottom, 1);

  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &DriveStatsPanel::refresh);
  timer->start(5000);
  refresh();
}

QFrame* DriveStatsPanel::card(const QString& title, QWidget* body) {
  QFrame* c = new QFrame(this);
  c->setObjectName("card");
  c->setStyleSheet("#card { border-radius: 12px; background-color: #1E1E1E; }");
  QVBoxLayout* l = new QVBoxLayout(c);
  l->setContentsMargins(24, 20, 24, 20);
  l->setSpacing(16);
  QLabel* t = new QLabel(title);
  t->setStyleSheet("font-size: 30px; font-weight: 700; color: white;");
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
  QString raw = readFile("/data/drive_stats/drive_stats.json");
  QJsonObject obj;
  if (!raw.isEmpty()) {
    QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8());
    if (doc.isObject()) obj = doc.object();
  }
  auto bucket = [&](const char* key, QLabel* km, QLabel* sub) {
    QJsonObject b = obj.value(key).toObject();
    double k = b.value("km").toDouble();
    int tr = int(b.value("trips").toDouble());
    double mn = b.value("min").toDouble();
    km->setText(QString::number(k, 'f', 1));
    sub->setText(QString("行程 %1 次 · 时长 %2 分").arg(tr).arg(QString::number(mn, 'f', 0)));
  };
  bucket("today", v_today_km, v_today_sub);
  bucket("week", v_week_km, v_week_sub);
  bucket("total", v_total_km, v_total_sub);
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
