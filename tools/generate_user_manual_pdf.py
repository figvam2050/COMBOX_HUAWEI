#!/usr/bin/env python3
"""generate_user_manual_pdf.py — Генератор PDF-інструкції користувача для COMBOX_HUAWEI.

Використовує ReportLab та системні шрифти Arial для бездоганного відображення
кирилиці (української мови), професійної верстки таблиць, колонтитулів та нумерації сторінок.
"""

import os
import sys
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_JUSTIFY, TA_RIGHT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, HRFlowable
)

# 1. Реєстрація шрифтів Arial з кирилицею
FONT_DIR = '/System/Library/Fonts/Supplemental'
pdfmetrics.registerFont(TTFont('Arial', os.path.join(FONT_DIR, 'Arial.ttf')))
pdfmetrics.registerFont(TTFont('Arial-Bold', os.path.join(FONT_DIR, 'Arial Bold.ttf')))
pdfmetrics.registerFont(TTFont('Arial-Italic', os.path.join(FONT_DIR, 'Arial Italic.ttf')))
pdfmetrics.registerFont(TTFont('Arial-BoldItalic', os.path.join(FONT_DIR, 'Arial Bold Italic.ttf')))


class NumberedCanvas(canvas.Canvas):
    """Канва для двопрохідної нумерації 'Сторінка X з Y' та колонтитулів."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_header_footer(num_pages)
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def draw_header_footer(self, page_count):
        self.saveState()
        self.setFont("Arial", 8)
        self.setFillColor(colors.HexColor("#64748B"))

        # Верхній колонтитул (крім першої сторінки)
        if self._pageNumber > 1:
            self.drawString(18 * mm, 285 * mm, "COM-Box (Huawei ESM Edition) — Інструкція користувача")
            self.setStrokeColor(colors.HexColor("#CBD5E1"))
            self.setLineWidth(0.5)
            self.line(18 * mm, 282 * mm, 192 * mm, 282 * mm)

        # Нижній колонтитул на всіх сторінках
        self.setStrokeColor(colors.HexColor("#CBD5E1"))
        self.setLineWidth(0.5)
        self.line(18 * mm, 15 * mm, 192 * mm, 15 * mm)

        self.drawString(18 * mm, 11 * mm, "Версія 1.0 (Жовтень 2026) | Ліцензія CC BY-NC-SA 4.0 | Не для комерційного продажу")
        page_str = f"Сторінка {self._pageNumber} з {page_count}"
        self.drawRightString(192 * mm, 11 * mm, page_str)
        self.restoreState()


def build_pdf(filename="USER_MANUAL.pdf"):
    doc = SimpleDocTemplate(
        filename,
        pagesize=A4,
        leftMargin=18 * mm,
        rightMargin=18 * mm,
        topMargin=20 * mm,
        bottomMargin=20 * mm
    )

    # Стилі
    styles = getSampleStyleSheet()

    c_primary = colors.HexColor("#1E3A8A")     # Темно-синій акцент
    c_secondary = colors.HexColor("#0D9488")   # Бірюзовий
    c_dark = colors.HexColor("#0F172A")        # Темний текст
    c_muted = colors.HexColor("#475569")       # Приглушений
    c_bg_box = colors.HexColor("#F8FAFC")      # Світло-сірий фон
    c_warn_bg = colors.HexColor("#FEF3C7")     # Світло-жовтий для уваги
    c_warn_border = colors.HexColor("#D97706") # Помаранчевий бордюр

    style_title = ParagraphStyle(
        'DocTitle',
        fontName='Arial-Bold',
        fontSize=20,
        leading=24,
        textColor=c_primary,
        alignment=TA_LEFT,
        spaceAfter=4
    )
    style_subtitle = ParagraphStyle(
        'DocSubTitle',
        fontName='Arial',
        fontSize=11,
        leading=15,
        textColor=c_secondary,
        alignment=TA_LEFT,
        spaceAfter=12
    )
    style_h1 = ParagraphStyle(
        'H1',
        fontName='Arial-Bold',
        fontSize=13,
        leading=17,
        textColor=c_primary,
        spaceBefore=14,
        spaceAfter=6,
        keepWithNext=True
    )
    style_h2 = ParagraphStyle(
        'H2',
        fontName='Arial-Bold',
        fontSize=10.5,
        leading=14,
        textColor=colors.HexColor("#334155"),
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=True
    )
    style_body = ParagraphStyle(
        'Body',
        fontName='Arial',
        fontSize=9,
        leading=13,
        textColor=c_dark,
        spaceAfter=5
    )
    style_bullet = ParagraphStyle(
        'Bullet',
        fontName='Arial',
        fontSize=9,
        leading=13,
        textColor=c_dark,
        leftIndent=12,
        spaceAfter=3
    )
    style_callout = ParagraphStyle(
        'Callout',
        fontName='Arial',
        fontSize=8.5,
        leading=12,
        textColor=colors.HexColor("#92400E")
    )
    style_th = ParagraphStyle(
        'TH',
        fontName='Arial-Bold',
        fontSize=8.5,
        leading=11,
        textColor=colors.white,
        alignment=TA_LEFT
    )
    style_td = ParagraphStyle(
        'TD',
        fontName='Arial',
        fontSize=8,
        leading=11,
        textColor=c_dark,
        alignment=TA_LEFT
    )
    style_td_bold = ParagraphStyle(
        'TDBold',
        fontName='Arial-Bold',
        fontSize=8,
        leading=11,
        textColor=c_dark,
        alignment=TA_LEFT
    )

    story = []

    # Шапка документа
    story.append(Paragraph("ІНСТРУКЦІЯ КОРИСТУВАЧА: COM-BOX", style_title))
    story.append(Paragraph("Шлюз-конвертер промислових батарей Huawei ESM (RS485 Modbus) → Deye CAN (Pylontech v2.0.6)", style_subtitle))
    story.append(HRFlowable(width="100%", thickness=1.5, color=c_primary, spaceBefore=0, spaceAfter=10))

    # Розділ 1: Призначення
    story.append(Paragraph("1. Загальний опис та призначення системи", style_h1))
    story.append(Paragraph(
        "Пристрій <b>COM-Box (Huawei ESM Edition)</b> — це мікроконтролерний шлюз на базі GD32F305RCT6, "
        "який забезпечує повну апаратну та протокольну сумісність між промисловими акумуляторами <b>Huawei ESM 48V (15S LiFePO4)</b> "
        "та сонячними гібридними інверторами <b>Deye</b> (Lithium Mode 00 = Pylontech CAN v2.0.6, 500 кбіт/с).",
        style_body
    ))
    story.append(Paragraph("<b>Основні функції пристрою:</b>", style_body))
    story.append(Paragraph("• <b>Трансляція протоколів:</b> опитує акумулятори Huawei по RS485 (Modbus RTU 9600 8N1) та щосекунди надсилає пакет із 6 кадрів CAN на інвертор Deye.", style_bullet))
    story.append(Paragraph("• <b>Агрегація банку:</b> зводить дані до 16 модулів у єдину віртуальну батарею (середня напруга, сумарний струм, середній SOC, мінімальний SOH по найгіршому блоку).", style_bullet))
    story.append(Paragraph("• <b>Динамічний захисний тейпер:</b> плавно знижує дозволений струм заряду (CCL) до 0 А при досягненні найвищою коміркою 3.550–3.600 В.", style_bullet))
    story.append(Paragraph("• <b>Захист від перерозряду:</b> знижує розрядний струм (DCL) на 50% при просіданні нижче 2.80 В і повністю вимикає розряд при 2.65 В.", style_bullet))
    story.append(Paragraph("• <b>Захист від холоду:</b> забороняє заряд при температурі нижче +2 °C (запобігання металізації літію).", style_bullet))
    story.append(Paragraph("• <b>Fail-Closed безпека:</b> при обриві зв'язку з батареями понад 4 секунди струми скидаються в 0 А, запобігаючи неконтрольованій роботі інвертора.", style_bullet))

    # Розділ 2: Роз'єми та розпіновка
    story.append(Paragraph("2. Роз'єми пристрою та розпіновка кабелів", style_h1))
    story.append(Paragraph("На панелі COM-Box розташовані три гнізда RJ45 та гвинтовий клемник живлення:", style_body))

    headers_ports = [Paragraph("Роз'єм", style_th), Paragraph("Призначення", style_th), Paragraph("Розпіновка (RJ45, піни 1..8)", style_th), Paragraph("Параметри лінії", style_th)]
    data_ports = [
        headers_ports,
        [Paragraph("<b>BMS-485</b>", style_td_bold), Paragraph("Підключення до порту COM батарей Huawei ESM", style_td), Paragraph("<b>Пін 1:</b> GND (сигнальна земля)<br/><b>Пін 5:</b> RS485-B (Data−)<br/><b>Пін 6:</b> RS485-A (Data+)", style_td), Paragraph("Modbus RTU, 9600 8N1,<br/>авто-напрямок", style_td)],
        [Paragraph("<b>PCS-CAN</b>", style_td_bold), Paragraph("Підключення до порту BMS інвертора Deye", style_td), Paragraph("<b>Пін 1:</b> GND<br/><b>Пін 7:</b> CAN-L<br/><b>Пін 8:</b> CAN-H", style_td), Paragraph("Pylontech CAN v2.0.6,<br/>500 кбіт/с, 1 Гц", style_td)],
        [Paragraph("<b>PCS-485</b>", style_td_bold), Paragraph("Діагностичний порт (дзеркало трафіку)", style_td), Paragraph("<b>Пін 1:</b> GND<br/><b>Пін 5:</b> RS485-B (Data−)<br/><b>Пін 6:</b> RS485-A (Data+)", style_td), Paragraph("Modbus RTU, 9600 8N1<br/>(дзеркало при DIP2=OFF)", style_td)],
        [Paragraph("<b>Power (B+/B−)</b>", style_td_bold), Paragraph("Гвинтовий клемник живлення COM-Box", style_td), Paragraph("<b>B+:</b> +15…+60 В DC (через запобіжник 1–2 А)<br/><b>B−:</b> GND (мінус батареї)", style_td), Paragraph("Споживання плати<br/>менше 0.5 Вт", style_td)],
    ]
    t_ports = Table(data_ports, colWidths=[28 * mm, 45 * mm, 62 * mm, 39 * mm])
    t_ports.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), c_primary),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E1")),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, c_bg_box]),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
    ]))
    story.append(t_ports)
    story.append(Spacer(1, 6))

    # Callout: GND обов'язковий
    warn_gnd = [
        [Paragraph("<b>⚠️ КРИТИЧНО ДЛЯ НАДІЙНОСТІ:</b> Контакт 1 (GND) обов'язково має бути з'єднаний спільним проводом між роз'ємом BMS-485 коробки та контактом 6 (або 8) порту COM батареї. Відсутність спільної землі призводить до накопичення статичного потенціалу від імпульсних завад інвертора та частих збоїв прийому RS485.", style_callout)]
    ]
    t_warn_gnd = Table(warn_gnd, colWidths=[174 * mm])
    t_warn_gnd.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), c_warn_bg),
        ('BOX', (0, 0), (-1, -1), 1, c_warn_border),
        ('TOPPADDING', (0, 0), (-1, -1), 5),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 5),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    story.append(t_warn_gnd)

    # Розділ 3: Підключення акумуляторів
    story.append(Paragraph("3. Схема підключення акумуляторів Huawei ESM (Daisy Chain)", style_h1))
    story.append(Paragraph(
        "1. Роз'єм <b>BMS-485</b> коробки з'єднується з портом <b>COM_IN</b> першого акумулятора Huawei кабельним відтинком (піни 6→A, 5→B, 1→GND).<br/>"
        "2. Якщо модулів декілька, вони з'єднуються послідовно ланцюжком: <b>COM_OUT</b> попереднього модуля → <b>COM_IN</b> наступного звичайним прямим патч-кордом RJ45.<br/>"
        "3. <b>ВСТАНОВЛЕННЯ CAN-ТЕРМІНАТОРА 120 ОМ:</b> У вільний роз'єм <b>COM_OUT останнього модуля</b> ланцюжка обов'язково встановлюється штатний CAN-термінатор 120 Ом (резистор між контактами 7 і 8).",
        style_body
    ))

    warn_term = [
        [Paragraph("<b>Чому термінатор 120 Ом обов'язковий:</b> Модулі Huawei ESM використовують виту пару CAN (піни 7 і 8) для <b>апаратної самоадресації</b>. При наявності термінатора модулі автоматично отримують адреси <b>214 (0xD6), 215 (0xD7), 216 (0xD8)…</b>. Без термінатора ВСІ модулі залишаються на адресі 214, що викликає електричні колізії та повну втрату зв'язку на RS485. Після встановлення термінатора перезапустіть живлення модулів.", style_callout)]
    ]
    t_warn_term = Table(warn_term, colWidths=[174 * mm])
    t_warn_term.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), c_warn_bg),
        ('BOX', (0, 0), (-1, -1), 1, c_warn_border),
        ('TOPPADDING', (0, 0), (-1, -1), 5),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 5),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    story.append(t_warn_term)

    # Розділ 4: Налаштування Deye
    story.append(Paragraph("4. Налаштування інвертора Deye", style_h1))
    story.append(Paragraph(
        "1. З'єднайте роз'єм <b>PCS-CAN</b> коробки з портом <b>BMS</b> інвертора Deye патч-кордом RJ45.<br/>"
        "2. На сенсорному екрані Deye відкрийте меню: <code>Gear (Налаштування) → Battery Setting → Battery Mode</code>.<br/>"
        "3. Оберіть тип: <b>Lithium</b>, і в полі протоколу вкажіть: <b>00</b> (Lithium Mode 00 = Pylontech CAN v2.0.6, 500 кбіт/с).<br/>"
        "4. Натисніть кнопку <b>OK</b> для збереження.<br/>"
        "5. <b>Контроль успішного зв'язку:</b> на головному екрані з'явиться точний відсоток SOC батареї. "
        "Аварія <code>F58 BMS comm fail</code> має зникнути протягом 3–5 секунд. "
        "У системному меню <code>Li-Bms Page</code> відобразяться напруга, струм банку та коректна кількість виявлених блоків.",
        style_body
    ))

    # Розділ 5: DIP-перемикачі
    story.append(Paragraph("5. Керування DIP-перемикачами на корпусі", style_h1))
    story.append(Paragraph("На бічній грані приладу розташований 5-позиційний перемикач DIP (налаштування зчитуються динамічно):", style_body))

    headers_dip = [Paragraph("Перемикач", style_th), Paragraph("Призначення", style_th), Paragraph("Положення OFF (Вимкнено)", style_th), Paragraph("Положення ON (Увімкнено)", style_th)]
    data_dip = [
        headers_dip,
        [Paragraph("<b>DIP 1 (SW1)</b>", style_td_bold), Paragraph("Профіль напруги заряду банку", style_td), Paragraph("<b>100% (54.0 В)</b> — повний номінальний заряд 15S", style_td), Paragraph("<b>90% (51.8 В)</b> — консервативний щадний режим", style_td)],
        [Paragraph("<b>DIP 2 (SW2)</b>", style_td_bold), Paragraph("Режим діагностичного порту PCS-485", style_td), Paragraph("<b>Пасивне дзеркало</b> обміну коробка ↔ BMS (штатно)", style_td), Paragraph("Тестовий міст до інвертора (дослідницький)", style_td)],
        [Paragraph("<b>DIP 3 (SW3)</b>", style_td_bold), Paragraph("Резервний перемикач", style_td), Paragraph("Вільний", style_td), Paragraph("Вільний", style_td)],
        [Paragraph("<b>DIP 4 (SW4)</b>", style_td_bold), Paragraph("Резервний перемикач", style_td), Paragraph("Вільний", style_td), Paragraph("Вільний", style_td)],
        [Paragraph("<b>DIP 5 (SW5)</b>", style_td_bold), Paragraph("Базовий ліміт струму на 1 модуль", style_td), Paragraph("<b>40 А</b> на модуль (м'який тепловий запас)", style_td), Paragraph("<b>50 А</b> на модуль (паспортний максимум)", style_td)],
    ]
    t_dip = Table(data_dip, colWidths=[28 * mm, 45 * mm, 50 * mm, 51 * mm])
    t_dip.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), c_primary),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E1")),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, c_bg_box]),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
    ]))
    story.append(t_dip)

    # Розділ 6: Світлодіоди
    story.append(Paragraph("6. Світлодіодна індикація (LED)", style_h1))
    story.append(Paragraph("Стан системи відображається 5 світлодіодами:", style_body))

    headers_led = [Paragraph("Світлодіод", style_th), Paragraph("Колір / Розташування", style_th), Paragraph("Нормальний стан", style_th), Paragraph("Значення та діагностика", style_th)]
    data_led = [
        headers_led,
        [Paragraph("<b>LED5 «Running»</b>", style_td_bold), Paragraph("Зелений, фасадний", style_td), Paragraph("<b>Постійно світиться</b>", style_td), Paragraph("Живлення подано, ядро мікроконтролера GD32F305 працює стабільно", style_td)],
        [Paragraph("<b>LED6 «BMS-485»</b>", style_td_bold), Paragraph("Зелений, фасадний", style_td), Paragraph("<b>Активно блимає</b>", style_td), Paragraph("Блимає під час передачі запиту та прийому відповідей від батарей Huawei", style_td)],
        [Paragraph("<b>LED4 «PCS-CAN»</b>", style_td_bold), Paragraph("Зелений, фасадний", style_td), Paragraph("<b>Блимає 1 раз на сек.</b>", style_td), Paragraph("Підтверджує щосекундну успішну відправку пакета із 6 кадрів на інвертор", style_td)],
        [Paragraph("<b>LED3 «PCS-485»</b>", style_td_bold), Paragraph("Зелений, фасадний", style_td), Paragraph("Блимає при підключенні", style_td), Paragraph("Блимає на кожному байті, який транслюється у діагностичне дзеркало", style_td)],
        [Paragraph("<b>LED1 «Balance»</b>", style_td_bold), Paragraph("Зелений, плата (всередині)", style_td), Paragraph("Вимкнений або блимає", style_td), Paragraph("Вимкнений = 100% (базовий); Блимає ~1 Гц = авто-сесія балансу 54 В; Світиться = DIP1 ON (режим 90%)", style_td)],
    ]
    t_led = Table(data_led, colWidths=[32 * mm, 38 * mm, 42 * mm, 62 * mm])
    t_led.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), c_primary),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E1")),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, c_bg_box]),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
    ]))
    story.append(t_led)

    # Розділ 7: Діагностика
    story.append(Paragraph("7. Діагностика та зняття логів через порт PCS-485 (Дзеркало)", style_h1))
    story.append(Paragraph(
        "При положенні <b>DIP2 = OFF</b> порт <b>PCS-485</b> транслює повну копію всього обміну між коробкою та акумуляторами "
        "(запити Modbus FC03 та сирі відповіді батарей). Це дозволяє логувати роботу без втручання в кабель BMS:<br/>"
        "1. Підключіть перетворювач USB-RS485 до гнізда <code>PCS-485</code> (пін 6 = A, пін 5 = B, пін 1 = GND).<br/>"
        "2. На комп'ютері запустіть будь-який серійний термінал (або скрипт <code>python3 tools/rs485_sniff.py</code>) на швидкості <b>9600 бод, 8N1</b>.<br/>"
        "3. Ви побачите потік телеметрії в реальному часі (напруги комірок, струми, прапорці захисту).",
        style_body
    ))

    # Розділ 8: Troubleshooting
    story.append(Paragraph("8. Пошук та усунення несправностей (Troubleshooting)", style_h1))

    headers_trouble = [Paragraph("Симптом", style_th), Paragraph("Ймовірна причина", style_th), Paragraph("Спосіб вирішення", style_th)]
    data_trouble = [
        headers_trouble,
        [Paragraph("<b>LED Running не світиться</b>", style_td_bold), Paragraph("Немає живлення на клемах B+/B−", style_td), Paragraph("Перевірте напругу батареї на гвинтовому клемнику (має бути 15–60 В) та запобіжник.", style_td)],
        [Paragraph("<b>LED BMS-485 не блимає</b>", style_td_bold), Paragraph("Повна тиша на лінії RS485 або модуль спить", style_td), Paragraph("1. Перевірте полярність A/B (піни 6 та 5 RJ45) та спільний GND (пін 1).<br/>2. Зачекайте до 15 секунд для завершення процедури Wake-up сплячого модуля.", style_td)],
        [Paragraph("<b>Помилка F58 на інверторі</b>", style_td_bold), Paragraph("Немає зв'язку по CAN-шині", style_td), Paragraph("1. Перевірте патч-корд від PCS-CAN до гнізда BMS інвертора (піни 7 CAN-L, 8 CAN-H).<br/>2. Переконайтеся, що на Deye увімкнено Lithium Mode 00.", style_td)],
        [Paragraph("<b>Колізії на RS485 / невірна кількість модулів</b>", style_td_bold), Paragraph("Відсутній термінатор 120 Ом на COM_OUT", style_td), Paragraph("Встановіть резистор 120 Ом між пінами 7 і 8 роз'єму COM_OUT останнього блоку та перезапустіть живлення батарейного банку.", style_td)],
        [Paragraph("<b>Струм заряду впав до 0 А раніше 100%</b>", style_td_bold), Paragraph("Спрацював захисний тейпер за коміркою", style_td), Paragraph("Штатний алгоритм безпеки: найвища комірка досягла 3.60 В. Коробка захищає осередок від перенапруги. Зачекайте завершення вирівнювання.", style_td)],
    ]
    t_trouble = Table(data_trouble, colWidths=[38 * mm, 46 * mm, 90 * mm])
    t_trouble.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), c_primary),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E1")),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, c_bg_box]),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
    ]))
    story.append(t_trouble)

    # Розділ 9: Правовий статус
    story.append(Paragraph("9. Правовий статус та безпека", style_h1))
    story.append(Paragraph(
        "• <b>Ліцензія CC BY-NC-SA 4.0:</b> Прошивка та документація є відкритими для вільного некомерційного використання "
        "(домашні СЕС, освітні та DIY цілі). Будь-який комерційний продаж пристроїв із цією прошивкою або платне надання послуг — <b>категорично заборонені</b>.<br/>"
        "• <b>Hardware Disclaimer:</b> Плата та корпус COM-Box розроблені стороннім виробником (Felicity Solar / OEM). Автор не пов'язаний з виробником заліза. "
        "Проєкт є незалежною аматорською альтернативною прошивкою для відновлення працездатності придбаного обладнання (Right to Repair).<br/>"
        "• <b>Відмова від відповідальності:</b> Прошивка надається «як є» (AS IS). Робота із силовими акумуляторними батареями становить підвищену небезпеку. "
        "Користувач несе повну особисту відповідальність за якість монтажу ліній та безпеку силової електроустановки.",
        style_body
    ))

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"PDF successfully built: {filename}")


if __name__ == '__main__':
    out_pdf = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "USER_MANUAL.pdf")
    build_pdf(out_pdf)
