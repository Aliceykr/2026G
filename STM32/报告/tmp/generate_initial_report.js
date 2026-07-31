const fs = require('fs');
const path = require('path');
const {
  AlignmentType, BorderStyle, Document, Footer, Header, HeadingLevel,
  ImageRun, PageBreak, PageNumber, Packer, Paragraph, Table, TableCell, TableOfContents,
  TableRow, TextRun, WidthType, VerticalAlign, ShadingType, PageOrientation,
  SectionType
} = require('docx');

const OUT = path.resolve(__dirname, '..', '..', '周期信号测量分析装置设计报告.docx');
const A4_WIDTH = 11906;
const A4_HEIGHT = 16838;
const CONTENT_WIDTH = 9360;
const BLUE = '1F4E79';
const LIGHT_BLUE = 'D9EAF7';
const LIGHT_GRAY = 'F2F2F2';
const SCHEMATIC_DIR = path.resolve(__dirname, 'schematics');
const PHOTO_DIR = path.resolve(__dirname, '..', 'photo');

const border = { style: BorderStyle.SINGLE, size: 4, color: '808080' };
const borders = { top: border, bottom: border, left: border, right: border };

function run(text, opts = {}) {
  return new TextRun({
    text,
    font: opts.font || '宋体',
    eastAsia: opts.eastAsia || opts.font || '宋体',
    size: opts.size || 21,
    bold: !!opts.bold,
    italics: !!opts.italics,
    color: opts.color || '000000',
    subScript: !!opts.subScript,
    superScript: !!opts.superScript,
  });
}

function p(text, opts = {}) {
  const children = Array.isArray(text) ? text : [run(text, opts)];
  return new Paragraph({
    children,
    alignment: opts.align || AlignmentType.JUSTIFIED,
    spacing: { before: opts.before || 0, after: opts.after ?? 100, line: opts.line || 360 },
    indent: opts.indent === false ? undefined : { firstLine: opts.firstLine ?? 420 },
    keepNext: !!opts.keepNext,
    pageBreakBefore: !!opts.pageBreakBefore,
  });
}

function title1(text) {
  return new Paragraph({
    text,
    heading: HeadingLevel.HEADING_1,
    pageBreakBefore: true,
    spacing: { before: 0, after: 220 },
  });
}

function title2(text) {
  return new Paragraph({ text, heading: HeadingLevel.HEADING_2, spacing: { before: 180, after: 100 }, keepNext: true });
}

function title3(text) {
  return new Paragraph({ text, heading: HeadingLevel.HEADING_3, spacing: { before: 120, after: 60 }, keepNext: true });
}

function formula(text, number) {
  return new Paragraph({
    children: [run(text, { font: 'Cambria Math', eastAsia: '宋体', size: 22 }), run(number ? `    （${number}）` : '', { font: '宋体', size: 20 })],
    alignment: AlignmentType.CENTER,
    spacing: { before: 100, after: 120, line: 300 },
  });
}

function bullet(text, level = 0) {
  return new Paragraph({
    children: [run(text)],
    bullet: { level },
    spacing: { after: 60, line: 330 },
  });
}

function cell(text, width, opts = {}) {
  const paras = Array.isArray(text) ? text : [new Paragraph({
    children: [run(String(text), { size: opts.size || 19, bold: !!opts.bold })],
    alignment: opts.align || AlignmentType.CENTER,
    spacing: { before: 40, after: 40, line: 280 },
  })];
  return new TableCell({
    children: paras,
    width: { size: width, type: WidthType.DXA },
    verticalAlign: VerticalAlign.CENTER,
    margins: { top: 70, bottom: 70, left: 90, right: 90 },
    borders,
    shading: opts.fill ? { type: ShadingType.CLEAR, fill: opts.fill } : undefined,
  });
}

function table(headers, rows, widths, caption) {
  const out = [];
  if (caption) out.push(new Paragraph({ children: [run(caption, { size: 19, bold: true })], alignment: AlignmentType.CENTER, spacing: { before: 100, after: 70 }, keepNext: true }));
  const trs = [new TableRow({ tableHeader: true, children: headers.map((h, i) => cell(h, widths[i], { bold: true, fill: LIGHT_BLUE })) })];
  for (const r of rows) trs.push(new TableRow({ children: r.map((v, i) => cell(v, widths[i], { fill: rows.indexOf(r) % 2 ? 'FAFAFA' : undefined })) }));
  out.push(new Table({ rows: trs, width: { size: widths.reduce((a, b) => a + b, 0), type: WidthType.DXA }, columnWidths: widths, alignment: AlignmentType.CENTER }));
  return out;
}

function diagramBox(text, width, fill = LIGHT_BLUE) {
  return cell([new Paragraph({ children: [run(text, { bold: true, size: 19 })], alignment: AlignmentType.CENTER, spacing: { before: 100, after: 100 } })], width, { fill });
}

function arrow(width = 420) { return cell('→', width, { size: 24 }); }

function figure(imagePath, caption, width, height, pageBreakBefore = false) {
  return [
    new Paragraph({
      children: [new ImageRun({ data: fs.readFileSync(imagePath), type: 'png', transformation: { width, height } })],
      alignment: AlignmentType.CENTER,
      spacing: { before: 100, after: 70 },
      pageBreakBefore,
    }),
    new Paragraph({
      children: [run(caption, { size: 19, bold: true })],
      alignment: AlignmentType.CENTER,
      spacing: { before: 0, after: 140 },
      keepNext: true,
    }),
  ];
}

const children = [];

// Cover
children.push(new Paragraph({ spacing: { before: 1000, after: 0 } }));
children.push(new Paragraph({ children: [run('周期信号测量分析装置', { font: '黑体', size: 44, bold: true, color: BLUE })], alignment: AlignmentType.CENTER, spacing: { before: 700, after: 260 } }));
children.push(new Paragraph({ children: [run('设计报告', { font: '黑体', size: 34, bold: true })], alignment: AlignmentType.CENTER, spacing: { after: 850 } }));
children.push(new Table({
  width: { size: 7200, type: WidthType.DXA }, columnWidths: [2200, 5000], alignment: AlignmentType.CENTER,
  rows: [
    new TableRow({ children: [cell('项目', 2200, { bold: true, fill: LIGHT_BLUE }), cell('内容', 5000, { bold: true, fill: LIGHT_BLUE })] }),
    new TableRow({ children: [cell('系统架构', 2200), cell('AD9238 + Cyclone IV FPGA + STM32F407', 5000)] }),
    new TableRow({ children: [cell('测量范围', 2200), cell('10 kHz～500 kHz周期信号', 5000)] }),
    new TableRow({ children: [cell('主要功能', 2200), cell('频率、谐波幅值与相位、Upp、Urms、时域波形、频谱显示', 5000)] }),
    new TableRow({ children: [cell('技术特点', 2200), cell('FPGA实时滤波、谐波幅相拟合、二维标定与相位重建', 5000)] }),
  ],
}));
children.push(new Paragraph({ spacing: { before: 1400 } }));
children.push(new Paragraph({ children: [run('2026年7月', { size: 24 })], alignment: AlignmentType.CENTER }));
children.push(new Paragraph({ children: [new PageBreak()] }));

children.push(new Paragraph({ text: '摘  要', heading: HeadingLevel.TITLE, alignment: AlignmentType.CENTER, spacing: { after: 260 } }));
children.push(p('本装置面向10 kHz～500 kHz周期信号的测量与分析，采用外部高速模数转换器AD9238、Cyclone IV FPGA与STM32F407构成分层处理架构。FPGA完成高速采样、三级级联积分梳状抽取、扩展定点精度、三节二阶低通滤波及双速率帧缓存；STM32完成4096点频谱分析、谱峰插值、基波与谐波识别、正弦最小二乘幅相拟合、二维幅频标定和通道相位响应补偿，并由重建波形求取峰峰值Upp，由各正交分量求取真有效值Urms。系统同时提供1周期或3周期时域波形、10～500 kHz频谱、显示屏交互和上位机连续数据输出。系统频谱分析采样率为1.25 MSPS，频率间隔约305.18 Hz。测试结果表明，典型基波、三次谐波和四次谐波组合下最大频率误差为9 Hz，参数处理时间约151 ms，三周期波形更新时间约291 ms；在叠加1 MHz、200 mVpp干扰时仍可稳定识别主要信号分量。系统建立了覆盖10～500 kHz、50～250 mVpp的二维幅值标定表与相位补偿表，可有效提高宽频带条件下的综合测量精度。'));
children.push(p([run('关键词：', { bold: true }), run('周期信号；FPGA数字滤波；频谱分析；谐波拟合；二维标定；峰峰值重建')], { indent: false }));
children.push(new Paragraph({ children: [new PageBreak()] }));

children.push(new Paragraph({ text: '目  录', heading: HeadingLevel.TITLE, alignment: AlignmentType.CENTER, spacing: { after: 220 } }));
children.push(new TableOfContents('目录', { hyperlink: true, headingStyleRange: '1-3' }));

children.push(title1('1 系统方案论证与选择'));
children.push(title2('1.1 任务需求分析'));
children.push(p('装置需要对10 kHz～500 kHz范围内的周期信号进行分析，在信号由基波和不超过两个谐波分量构成时，给出基波频率、各有效分量幅值与相位、总波形峰峰值Upp和真有效值Urms，并显示时域波形与频谱。输入端采用50 Ω匹配口径，还需考虑高频小信号、非整周期采样、频谱泄漏、通道幅相响应以及1 MHz以上强干扰等因素。'));
children.push(title2('1.2 方案一：STM32H7片内双ADC高速采样'));
children.push(p('前期方案采用STM32H743片内ADC1和ADC2交错采样，目标采样率约8.192 MSa/s、单帧16384点，抽取后执行4096点FFT，并结合频率网格细化和多分量最小二乘拟合。该方案集成度高、硬件结构简单，适合验证频谱分析、幅相拟合和多帧统计等基本方法。'));
children.push(p('实测表明，该路线在高频小信号条件下受到片内ADC的INL/DNL、参考电压噪声、输入带宽，以及双ADC通道增益、偏置和采样时刻失配的共同限制。交错失配会形成确定性杂散，多帧平均虽可改善随机重复性，却无法消除系统性误差。此外，固定谐波模型难以完整覆盖基波与任意两个谐波的组合，因此未作为最终方案。'));
children.push(title2('1.3 方案二：外部高速ADC与STM32直接处理'));
children.push(p('采用独立高速ADC可显著改善模拟精度和动态范围，并避免片内双ADC交错失配；但若全部高速采样数据直接送入STM32，接口吞吐、缓存容量、实时数字滤波和干扰抑制压力较大。特别是强带外干扰存在时，仅依赖MCU软件后处理容易发生混叠或压缩有效动态范围。'));
children.push(title2('1.4 方案三：AD9238、FPGA与STM32分层处理'));
children.push(p('最终方案由AD9238完成高速采样，Cyclone IV FPGA执行实时抽取与低通滤波，并分别形成1.25 MSPS分析帧和5 MSPS波形帧；STM32F407负责FFT、谐波参数估计、标定换算、Upp/Urms计算和人机交互。该结构把确定性、高吞吐的前端处理放在FPGA，把复杂且便于迭代的浮点分析放在MCU，兼顾抗干扰能力、算法灵活性和工程可维护性。'));
children.push(...table(['比较项目', 'STM32H7双ADC', '外部ADC直连MCU', 'AD9238+FPGA+STM32'], [
  ['模拟精度', '受片内ADC与交错失配限制', '较高', '较高'],
  ['带外干扰抑制', '主要依赖软件', 'MCU负担较重', 'FPGA实时滤波'],
  ['任意谐波组合', '早期仅适合固定谐波组合', '可实现', '支持基波及任意两个谐波'],
  ['数据吞吐压力', '高', '高', 'FPGA抽取后较低'],
  ['算法迭代', '方便', '方便', 'MCU侧方便'],
  ['最终选择', '预研后弃用', '未采用', '采用'],
], [1800, 2400, 2400, 2760], '表1-1 三种总体方案比较'));

children.push(title1('2 理论分析与参数计算'));
children.push(title2('2.1 周期信号数学模型'));
children.push(p('设被测信号由基波及至多两个谐波组成，可统一表示为：'));
children.push(formula('u(t)=Σ Aₖ sin(2πk f₀t+φₖ)', '2-1'));
children.push(p('式中，f₀为基波频率，k为谐波次数，Aₖ为第k次分量的峰值幅度，φₖ为相对于统一正弦基准的相位。系统不预设谐波一定为2次或3次，而是在10～500 kHz有效频带内识别幅值最大的有效分量并匹配到整数谐波阶次，最多保留3个分量。'));
children.push(title2('2.2 采样率与频率分辨率'));
children.push(p('FPGA输出4096点分析帧，分析采样率为1.25 MSPS，因此FFT原始频率间隔为：'));
children.push(formula('Δf=Fₛ/N=1.25×10⁶/4096≈305.18 Hz', '2-2'));
children.push(p('305.18 Hz小于500 Hz最小分辨率要求。进一步通过窗函数抑制泄漏、谱峰插值和正弦拟合对栅格内频率进行细化，使频率测量不局限于整数FFT频点。波形显示路径采用5 MSPS采样率，以提高100 kHz及更高频率信号的相位连续性和显示平滑度。'));
children.push(title2('2.3 频谱泄漏与谱峰插值'));
children.push(p('实际采样帧通常不包含整数个周期，直接FFT会产生谱泄漏。系统对去直流后的数据加窗，再在候选谱峰附近进行亚频点插值，获得更准确的中心频率和初始幅值。基波频率确定后，各候选峰按f≈kf₀进行谐波匹配，避免把1 MHz干扰或其他非整数倍杂散误认为有效谐波。'));
children.push(title2('2.4 正弦最小二乘幅相估计'));
children.push(p('对每个已识别频率，将正弦分量写成线性形式：'));
children.push(formula('x[n]=a sin(ωn)+b cos(ωn)+c', '2-3'));
children.push(p('由最小二乘求得a、b和直流项c，分量峰值A及相位φ分别为：'));
children.push(formula('A=√(a²+b²)，  φ=atan2(b,a)', '2-4'));
children.push(p('该方法利用整帧样本联合估计，比直接读取单个FFT幅值具有更小的栅栏效应。为进一步降低频率偏差和带外干扰引起的拟合误差，参数估计过程还结合频率细化与干扰分量联合拟合，提高复杂输入条件下的稳定性。'));
children.push(title2('2.5 真有效值计算'));
children.push(p('不同整数次谐波在整周期内相互正交，去除直流分量后，总信号真有效值为各分量有效值平方和开方：'));
children.push(formula('Uᵣₘₛ=√[(1/2)ΣAₖ²]', '2-5'));
children.push(p('因此Urms只与各分量峰值有关，不受初相位影响。若分量幅值为信号源50 Ω端口的峰值口径，计算时必须保持相同口径，避免把峰值、峰峰值或High-Z显示值混用。'));
children.push(title2('2.6 峰峰值Upp的相位重建算法'));
children.push(p('含谐波波形的正峰值与负峰值通常不会等于各分量峰值的简单代数和。系统使用标定后的幅值和相位重建一个基波周期内的连续波形，并求其最大值与最小值：'));
children.push(formula('Upp=max u(t)−min u(t)', '2-6'));
children.push(p('为降低离散扫描遗漏极值造成的误差，可先进行高密度相位扫描定位候选极值，再用局部插值或迭代求导数零点。相位校准尤其重要：绝对起始相位随采样触发时刻变化，但满足φₖ−kφ₁的相对组合相位在相同信号条件下应保持稳定。通道相位补偿应作为绝对频率f的连续响应P(f)进行解包和插值，而不是为每一种谐波组合单独建表。'));
children.push(title2('2.7 二维幅值标定'));
children.push(p('模拟前端和数字滤波的增益同时随频率与幅度变化，一维比例系数难以保证全量程精度。系统采用二维幅值标定：频率轴覆盖10～500 kHz、步进10 kHz；幅度轴采用50、100、150、200、250 mVpp五档。每个标定点采集20帧并取中位数，测量时先沿幅度轴分段插值，再沿频率轴插值，直接得到50 Ω负载条件下的峰值电压。'));

children.push(title1('3 系统硬件设计'));
children.push(title2('3.1 总体硬件结构'));
children.push(new Table({ width: { size: CONTENT_WIDTH, type: WidthType.DXA }, columnWidths: [1500, 420, 1650, 420, 1800, 420, 1800, 420, 930], rows: [new TableRow({ children: [
  diagramBox('BNC输入\n50 Ω匹配', 1500), arrow(), diagramBox('AD9238\n高速ADC', 1650), arrow(), diagramBox('Cyclone IV\n抽取与低通', 1800), arrow(), diagramBox('STM32F407\n测量与控制', 1800), arrow(), diagramBox('显示屏/\n上位机', 930)
] })] }));
children.push(new Paragraph({ children: [run('图3-1 系统硬件总体框图', { size: 19, bold: true })], alignment: AlignmentType.CENTER, spacing: { before: 70, after: 120 } }));
children.push(title2('3.2 输入与模数转换模块'));
children.push(p('输入端按50 Ω系统口径设计，信号源选择50 Ω负载模式。AD9238对输入信号进行高速采样，FPGA将模数转换器输出转换为以零为中心的有符号数字量。外部ADC方案提供了比片内ADC更稳定的动态范围，并为小幅度高频信号的二维标定提供硬件基础。'));
children.push(p('模拟前端采用AD8065与AD8138构成单端至差分信号调理电路，并以AD9238共模电平驱动差分输入。ADC模拟电源与数字电源分别去耦，数字输出经串联电阻连接至FPGA，以降低高速边沿引起的反射和串扰。'));
children.push(...figure(path.join(SCHEMATIC_DIR, 'adda_4.png'), '图3-2 AD9238输入调理及模数转换电路', 620, 384));
children.push(title2('3.3 FPGA模块'));
children.push(p('FPGA采用Cyclone IV EP4CE6E22C8。数据通路依次完成模数转换数据接收、三级级联积分梳状滤波器十倍抽取、定点精度扩展、三节二阶低通滤波、双速率帧缓存和串行传输。参数分析通道对滤波结果进一步4倍抽取，形成1.25 MSPS、4096点分析帧；波形显示通道直接保存5 MSPS滤波结果。两个2048×16 bit存储器组成一个4096×16 bit逻辑帧，采集与读取之间采用可靠的跨时钟域握手机制。'));
children.push(...figure(path.join(SCHEMATIC_DIR, 'adda_2.png'), '图3-3 Cyclone IV FPGA主体及高速数据接口电路', 590, 560));
children.push(p('FPGA最小系统还包括50 MHz有源晶振、上电复位、JTAG下载接口和M25P16配置存储器。时钟信号经串联电阻接入FPGA全局时钟端，配置与复位端设置上拉或下拉电阻，保证系统上电后进入确定状态。'));
children.push(...figure(path.join(SCHEMATIC_DIR, 'adda_3.png'), '图3-4 FPGA时钟、复位、JTAG及配置存储电路', 620, 428));
children.push(title2('3.4 STM32F407与通信接口'));
children.push(p('STM32F407通过高速串行接口控制FPGA启动参数分析或波形采集，并读取采集状态与样本数据。串口屏用于显示时域波形、频谱图和测量参数，调试接口可连续输出测量结果，便于标定和性能测试。屏幕按键分别控制时域测量、频域测量和波形周期切换。'));
children.push(p('主控板以STM32F407ZGT6为核心，配置独立晶振、复位、程序下载调试接口、外部存储器和显示接口，并通过排针与FPGA板连接。各并行接口串联电阻用于改善信号完整性，主控板预留的其他扩展接口不参与本题核心测量功能。'));
children.push(...figure(path.join(PHOTO_DIR, 'b70b3ff0631bc01bb55f9d1709f8747f.png'), '图3-5 STM32F407主控板及FPGA连接接口电路', 620, 432));
children.push(title2('3.5 电源与抗干扰设计'));
children.push(p('系统采用单路5 V输入，各数字与模拟电源由板上稳压电路产生。高速ADC、FPGA和MCU应采用分区布线、完整地平面、就近去耦和时钟回流控制；模拟输入走线保持阻抗连续并远离数字时钟。FPGA低通滤波用于抑制1 MHz及以上干扰，但模拟前端仍需保证输入不过载，避免强干扰在ADC前端产生非线性互调。'));
children.push(p('电源模块由单路5 V输入产生ADC、FPGA及扩展电路所需电源，其中FPGA使用3.3 V、2.5 V和1.2 V多路电源，模拟调理电路使用正负电源。各稳压器输入、输出端均配置电解电容和高频陶瓷电容，ADC与FPGA电源在器件附近进一步进行分布式去耦。'));
children.push(...figure(path.join(SCHEMATIC_DIR, 'adda_1.png'), '图3-6 系统电源转换与滤波电路', 620, 437));

children.push(title1('4 FPGA与STM32软件设计'));
children.push(title2('4.1 FPGA采集与滤波流程'));
children.push(new Table({ width: { size: 8000, type: WidthType.DXA }, columnWidths: [8000], alignment: AlignmentType.CENTER, rows: [
  new TableRow({ children: [diagramBox('AD9238采样与码型转换', 8000)] }),
  new TableRow({ children: [diagramBox('↓', 8000, 'FFFFFF')] }),
  new TableRow({ children: [diagramBox('三级抽取滤波 + 定点精度扩展', 8000)] }),
  new TableRow({ children: [diagramBox('↓', 8000, 'FFFFFF')] }),
  new TableRow({ children: [diagramBox('三节二阶低通滤波', 8000)] }),
  new TableRow({ children: [diagramBox('↓', 8000, 'FFFFFF')] }),
  new TableRow({ children: [diagramBox('分析模式1.25 MSPS / 波形模式5 MSPS', 8000)] }),
  new TableRow({ children: [diagramBox('↓', 8000, 'FFFFFF')] }),
  new TableRow({ children: [diagramBox('4096点双存储区缓存，送往STM32', 8000)] }),
] }));
children.push(new Paragraph({ children: [run('图4-1 FPGA数据处理流程', { size: 19, bold: true })], alignment: AlignmentType.CENTER, spacing: { before: 70, after: 120 } }));
children.push(title2('4.2 STM32测量状态机'));
children.push(p('STM32控制流程包括空闲、参数采集、波形采集、结果计算和显示保持等状态。一次参数采集完成后依次执行频谱分析、标定换算、相位补偿、Upp与Urms计算及数据输出；时域测量还会启动独立的5 MSPS波形采集。连续测试模式可在设定间隔后自动开始下一次测量，单次采集超时上限为2 s。'));
children.push(...table(['阶段', '主要处理', '输出'], [
  ['启动', '读取FPGA ID、初始化屏幕与串口', '启动状态'],
  ['分析采集', '获取1.25 MSPS、4096点帧', '原始分析数据'],
  ['参数提取', '频谱变换、插值、谐波匹配、最小二乘', '频率、数字幅度和相位'],
  ['标定换算', '二维幅值插值、相位响应补偿', 'mV与校准相位'],
  ['综合计算', '真Urms、相位重建Upp', '总参数'],
  ['显示与输出', '按模式更新显示屏并输出测量数据', '时域、频域与参数'],
], [1350, 5000, 3010], '表4-1 STM32参数处理流程'));
children.push(title2('4.3 频谱分析与谐波识别'));
children.push(p('频谱分析首先对固定长度采样帧进行去直流、加窗和快速傅里叶变换，再搜索有效频带内的谱峰并进行频率插值。系统记录基波频率、各分量的谐波阶次、绝对频率、幅度和相位，最多保留基波及两个有效谐波。频谱显示覆盖10～500 kHz，并通过分区最大值提取将频谱压缩为适合显示屏绘制的点数。'));
children.push(title2('4.4 标定换算与Upp计算'));
children.push(p('各频率分量的数字幅度通过二维标定曲线换算为峰值电压，通道相位响应则按绝对频率进行连续插值补偿。标定结果统一采用50 Ω负载条件下的电压口径。完成幅度和相位校准后，系统根据各有效分量重建一个基波周期内的波形，并搜索最大值与最小值求得Upp；Urms由各正交分量的有效值平方和开方得到。'));
children.push(title2('4.5 串口屏交互逻辑'));
children.push(...table(['按键名称', '主要功能', '显示内容', '工作关系'], [
  ['时域测量', '启动参数测量与波形采集', '基频、Upp、Urms及时域波形', '不改变频域显示'],
  ['周期切换', '选择显示1个或3个完整周期', '仅改变时域波形跨度', '不影响频谱显示'],
  ['频域测量', '启动参数测量与频谱分析', '各分量频率、幅值及频谱图', '不改变时域波形'],
  ['量程切换', '切换辅助显示模式', '调整界面显示状态', '不影响主要测量流程'],
], [1500, 2800, 3100, 1960], '表4-2 显示屏按键功能'));
children.push(p('时域测量和频域测量采用相互独立的显示路径，执行任一测量不会覆盖另一页面的波形内容。周期切换只改变时域波形的显示跨度，不影响频谱图。电压幅值采用三位小数显示，以便观察标定前后的细微变化。'));

children.push(title1('5 系统测试方案与结果'));
children.push(title2('5.1 测试仪器与连接'));
children.push(p('测试时由信号发生器输出单通道合成波形，负载模式设为50 Ω，经BNC连接至装置输入端；上位机记录连续测量数据，显示屏用于观察时域波形、频谱图和参数值。多谐波及干扰信号均由信号发生器的谐波合成功能产生，避免外部合路器引入额外的幅相误差。'));
children.push(title2('5.2 功能测试项目'));
children.push(...table(['序号', '测试项目', '建议测试点', '判据'], [
  ['1', '频率范围', '10、50、100、200、300、400、500 kHz', '频率误差满足题目要求'],
  ['2', '幅值范围', '50、100、150、200、250 mVpp', '分量幅值及Upp误差达标'],
  ['3', '单谐波组合', '基波与一个谐波，改变阶次和相位', '正确识别阶次、幅相'],
  ['4', '双谐波组合', '基波与任意两个谐波', '三个分量均正确'],
  ['5', '强干扰', '叠加1 MHz/200 mVpp及2 MHz干扰', '有效分量和综合值稳定'],
  ['6', '波形显示', '1周期/3周期、低频/高频', '方向正确、平滑、无明显畸变'],
  ['7', '响应时间', '参数、时域、频域按钮', '总响应时间小于2 s'],
  ['8', '无信号', '输入端无有效信号', '正确显示无信号状态'],
], [700, 1600, 4200, 2860], '表5-1 系统测试项目'));
children.push(title2('5.3 典型多谐波测试'));
children.push(p('典型组合测试采用10.5 kHz、120 mVpp基波，31.5 kHz、60 mVpp三次谐波和42 kHz、40 mVpp四次谐波。按峰值口径，各分量为60 mV、30 mV和20 mV；理论Urms约49.50 mV，相位重建理论Upp约154.89 mV。'));
children.push(...table(['参数', '理论值', '实机结果', '备注'], [
  ['基波/三次/四次谐波峰值', '60/30/20 mV', '60/30/20 mV', '二维标定口径'],
  ['Upp', '约154.89 mV', '157 mV', '实测结果'],
  ['Urms', '约49.50 mV', '50 mV', '实测结果'],
  ['最大频率误差', '—', '9 Hz', '典型组合'],
  ['参数处理时间', '<2 s', '约151 ms', '不含人工操作'],
  ['3周期波形时间', '<2 s', '约291 ms', '满足响应要求'],
], [2000, 2200, 2200, 2960], '表5-2 典型多谐波组合测试结果'));
children.push(title2('5.4 1 MHz强干扰测试'));
children.push(p('抗干扰测试设置100 kHz、120 mVpp基波，300 kHz、60 mVpp三次谐波和400 kHz、40 mVpp四次谐波，并叠加1 MHz、200 mVpp单频干扰。实测得到三个有效分量的峰值分别为60 mV、30 mV和20 mV，Upp为154 mV，Urms为49 mV，参数处理时间约151 ms，结果表明FPGA抽取低通链对带外强干扰具有明显抑制作用。'));
children.push(p('1 MHz、200 mVpp为代表性干扰测试点，系统性能还需结合不同干扰频率、幅度和相位进行综合评价。若强干扰导致模拟前端或ADC饱和，数字滤波无法恢复已经失真的有效信号。'));
children.push(title2('5.5 多组综合测量精度'));
children.push(p('选取不同基波频率、谐波阶次、幅值和相位组合进行综合测试。第4组中的600 kHz分量和第5组中的1 MHz分量超出有效测量频带，理论整体参数按滤除带外分量后的有效信号计算。'));
children.push(...table(['组别', '信号组成（频率/峰值/相位差）', '理论Urms', '实测Urms', '理论Upp', '实测Upp'], [
  ['1', '10 kHz/50 mV；二次20 kHz/25 mV/30°；十次100 kHz/25 mV/120°', '43.301 mV', '43.200 mV', '169.348 mV', '169.357 mV'],
  ['2', '50 kHz/50 mV；三次150 kHz/30 mV/60°；十次500 kHz/32.5 mV/150°', '47.202 mV', '47.193 mV', '195.974 mV', '196.036 mV'],
  ['3', '100 kHz/50 mV；二次200 kHz/40 mV/90°；五次500 kHz/42.5 mV/240°', '54.343 mV', '54.377 mV', '198.516 mV', '198.540 mV'],
  ['4', '100 kHz/75 mV；二次200 kHz/60 mV/78.2°；600 kHz带外分量', '67.915 mV', '67.936 mV', '213.632 mV', '213.744 mV'],
  ['5', '100 kHz/75 mV；二次200 kHz/60 mV/78.2°；1 MHz带外干扰', '67.915 mV', '67.932 mV', '213.632 mV', '213.700 mV'],
], [650, 3910, 1200, 1200, 1200, 1200], '表5-3 多组综合测量结果'));
children.push(p('相对误差按实测值与理论值之差的绝对值除以理论值计算，测量准确度取100%减去相对误差。各组误差与准确度见表5-4。'));
children.push(new Paragraph({ children: [new PageBreak()] }));
children.push(...table(['组别', 'Urms绝对误差', 'Urms相对误差', 'Urms准确度', 'Upp绝对误差', 'Upp相对误差', 'Upp准确度'], [
  ['1', '0.101 mV', '0.234%', '99.766%', '0.009 mV', '0.005%', '99.995%'],
  ['2', '0.009 mV', '0.020%', '99.980%', '0.062 mV', '0.031%', '99.969%'],
  ['3', '0.034 mV', '0.063%', '99.937%', '0.024 mV', '0.012%', '99.988%'],
  ['4', '0.021 mV', '0.030%', '99.970%', '0.112 mV', '0.052%', '99.948%'],
  ['5', '0.017 mV', '0.024%', '99.976%', '0.068 mV', '0.032%', '99.968%'],
], [650, 1450, 1350, 1350, 1450, 1350, 1350], '表5-4 综合测量误差与准确度'));
children.push(p('五组测试中，Urms平均相对误差为0.074%，平均准确度为99.926%；Upp平均相对误差为0.027%，平均准确度为99.973%。全部十项电压结果的平均准确度为99.950%，最小准确度为99.766%。最大Urms绝对误差为0.101 mV，最大Upp绝对误差为0.112 mV，均明显小于题目规定的5 mV误差限值。'));
children.push(title2('5.6 全量程精度验证方法'));
children.push(p('为验证题目规定的电压测量误差，应采用独立测试点遍历频率、总幅度、谐波阶次、相位和干扰条件，并记录信号源设定值、理论计算值与装置显示值。测试结果分别统计最大绝对误差、均方根误差、95%分位误差和重复性，避免仅使用标定节点评价系统性能。'));
children.push(...table(['测试维度', '取值范围', '测试目的'], [
  ['基波频率', '10～500 kHz，至少每10 kHz', '覆盖标定节点与节点中点'],
  ['分量幅值', '峰峰值50～250 mV范围内多档', '包含低幅和满量程边界'],
  ['谐波阶次', '二次、三次、四次及其他有效阶次', '总分量数不超过3'],
  ['相位', '0°～330°，步进30°或随机', '验证相位重建极值'],
  ['干扰', '无干扰、1 MHz/200 mVpp、2 MHz等', '检查滤波和混叠路径'],
  ['复测', '每点至少20帧', '评估中位数、均值和波动'],
], [1800, 4000, 3560], '表5-5 全量程测试矩阵'));
children.push(p('全量程精度按照表5-5所列矩阵进行独立测试，并以完整测试记录、误差统计及最坏测试点结果作为性能判定依据。'));

children.push(title1('6 误差来源与改进措施'));
children.push(title2('6.1 模拟与采样误差'));
children.push(bullet('信号源幅值口径和50 Ω负载误差：必须确认设置值是峰值、峰峰值还是High-Z等效值。'));
children.push(bullet('输入网络、ADC驱动及AD9238的频率响应、增益误差、偏置、噪声和非线性。'));
children.push(bullet('时钟抖动在高频分量上的相位噪声，以及电源、地回流和数字串扰。'));
children.push(title2('6.2 数字处理误差'));
children.push(bullet('非整周期采样产生频谱泄漏，谱峰插值和拟合频率偏差会进一步影响幅相。'));
children.push(bullet('FPGA抽取滤波和二阶低通滤波中的定点运算截断、饱和及通带幅相响应；扩展定点精度可降低级间舍入误差。'));
children.push(bullet('谐波阶次错误、强干扰误识别或混叠分量未纳入联合拟合。'));
children.push(bullet('Upp极值搜索网格过疏会漏过真实极值，应使用局部细化或解析导数求根。'));
children.push(title2('6.3 标定误差'));
children.push(p('二维幅值表可以补偿稳定、可重复的频率与幅度响应，但无法补偿随温度、电源、连接器和器件批次漂移的误差。相位数据需要先解包，并去除任意线性时延后形成连续响应曲线；若直接记录每次采集的绝对相位，触发时刻变化会使标定失效。标定数据应与系统配置对应，并完整记录标定日期、测试仪器和环境温度。'));
children.push(title2('6.4 后续优化方向'));
children.push(bullet('在标定节点之间增加独立验证点，避免只验证插值表本身。'));
children.push(bullet('将Upp重建的极值搜索改为“粗扫描+导数求根/抛物线细化”，并使用双精度离线对照。'));
children.push(bullet('对1 MHz及2 MHz强干扰建立专门的混叠与前端过载测试，必要时调整模拟低通。'));
children.push(bullet('增加板温采样，评估温漂后决定是否需要分温区标定。'));
children.push(bullet('测试过程中同时记录原始测量值和校准后测量值，提高结果可追溯性。'));

children.push(title1('7 结论'));
children.push(p('本设计经过纯MCU高速采样方案预研后，最终形成AD9238、Cyclone IV FPGA与STM32F407分层处理架构。FPGA完成实时抽取和低通滤波，STM32完成频谱分析、任意谐波组合识别、正弦最小二乘幅相估计、二维幅值标定、相位补偿、真Urms与相位重建Upp计算，并实现时域和频域独立显示及连续数据输出。测试结果验证了10～500 kHz分析范围、典型多谐波组合、1 MHz强干扰抑制和小于2 s的响应速度。'));
children.push(p('系统具备进一步提高测量精度的算法与标定基础。综合性能通过全量程、多相位、多谐波及强干扰条件下的独立测试进行确认，最终结论以完整测试记录和统计结果为依据。'));

children.push(title1('参考文献'));
[
  '[1] 全国大学生电子设计竞赛组委会. 周期信号测量分析装置（G题）[Z].',
  '[2] Analog Devices. AD9238: Dual 12-Bit, 20/40/65 MSPS ADC Data Sheet[Z].',
  '[3] STMicroelectronics. STM32F407xx Reference Manual[Z].',
  '[4] Intel. Cyclone IV Device Handbook[Z].',
  '[5] Oppenheim A V, Schafer R W. Discrete-Time Signal Processing[M].',
].forEach(x => children.push(p(x, { indent: false, align: AlignmentType.LEFT, after: 80 })));

children.push(title1('附录A 系统方案演进摘要'));
children.push(...table(['研究方案', '主要技术路线', '方案结论'], [
  ['纯MCU高速采样方案', '双ADC交错采样、DFT/FFT分析、多轮幅相拟合、缓存与DMA优化', '用于前期技术验证，受片内ADC精度与交错失配限制，未作为最终方案'],
  ['FPGA+STM32分层方案', '外部高速ADC、FPGA抽取滤波、STM32幅相估计与二维标定', '作为最终系统方案，兼顾测量精度、抗干扰能力和算法扩展性'],
], [2400, 4260, 2700], '表A-1 系统方案演进'));
children.push(p('系统方案的改进重点是将误差控制从单纯依赖后端算法平均，扩展为外部高速ADC保证采样基础、FPGA前置滤波抑制带外干扰、STM32完成参数估计与二维标定的完整测量链。该架构能够支持基波与任意两个有效谐波的组合，而不局限于固定的谐波阶次。'));

children.push(title1('附录B 板载扩展电路'));
children.push(p('采集板同时配置AD9767双通道数模转换器及输出调理电路，用于通用信号输出与调试扩展。本装置只使用AD9238采样通道，数模转换器在测量过程中保持静止输出，不参与被测信号的产生、测量或参数计算。'));
children.push(...figure(path.join(SCHEMATIC_DIR, 'adda_5.png'), '图B-1 AD9767数模转换及输出调理扩展电路', 620, 364));

const doc = new Document({
  creator: '周期信号测量分析装置项目组',
  title: '周期信号测量分析装置设计报告',
  description: '周期信号测量分析装置技术报告',
  styles: {
    default: {
      document: { run: { font: '宋体', eastAsia: '宋体', size: 21 }, paragraph: { spacing: { line: 360 } } },
      heading1: { run: { font: '黑体', eastAsia: '黑体', size: 30, bold: true, color: BLUE }, paragraph: { spacing: { before: 0, after: 220 }, outlineLevel: 0 } },
      heading2: { run: { font: '黑体', eastAsia: '黑体', size: 25, bold: true }, paragraph: { spacing: { before: 180, after: 100 }, outlineLevel: 1 } },
      heading3: { run: { font: '黑体', eastAsia: '黑体', size: 22, bold: true }, paragraph: { spacing: { before: 120, after: 60 }, outlineLevel: 2 } },
      title: { run: { font: '黑体', eastAsia: '黑体', size: 32, bold: true }, paragraph: { alignment: AlignmentType.CENTER } },
    },
  },
  sections: [{
    properties: {
      type: SectionType.CONTINUOUS,
      page: { size: { width: A4_WIDTH, height: A4_HEIGHT, orientation: PageOrientation.PORTRAIT }, margin: { top: 1417, right: 1276, bottom: 1276, left: 1276, header: 600, footer: 600 } },
    },
    headers: { default: new Header({ children: [new Paragraph({ children: [run('周期信号测量分析装置设计报告', { size: 18, color: '666666' })], alignment: AlignmentType.CENTER, border: { bottom: { style: BorderStyle.SINGLE, size: 4, color: 'BFBFBF' } } })] }) },
    footers: { default: new Footer({ children: [new Paragraph({ children: [run('— ', { size: 18 }), new TextRun({ children: [PageNumber.CURRENT], font: 'Times New Roman', size: 18 }), run(' —', { size: 18 })], alignment: AlignmentType.CENTER })] }) },
    children,
  }],
});

Packer.toBuffer(doc).then(buffer => {
  fs.writeFileSync(OUT, buffer);
  process.stdout.write(`${OUT}\n${buffer.length}\n`);
});
